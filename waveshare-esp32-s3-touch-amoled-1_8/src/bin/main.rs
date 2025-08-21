#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use bevy_ecs::prelude::*;
use core::fmt::Write;
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_graphics_framebuf::FrameBuf;
use embedded_graphics_framebuf::backends::FrameBufferBackend;
use heapless::String;

use esp_hal::Blocking;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::spi::Mode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::time::Rate;
use log::info;

use esp_hal::rng::Rng;
use esp_println::logger::init_logger_from_env;
use esp_println::println;

// Custom panic handler for better debugging
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Print panic information
    println!("\n=== PANIC OCCURRED ===");

    // Print panic location if available
    if let Some(location) = info.location() {
        println!(
            "Panic occurred at {}:{}:{}",
            location.file(),
            location.line(),
            location.column()
        );
    } else {
        println!("Panic occurred at unknown location");
    }

    // Print panic message if available
    let message = info.message();
    println!("Panic message: {}", message);

    // Print memory information
    println!("\n=== MEMORY INFO ===");
    println!("Stack pointer: unavailable (assembly removed)");

    // Print some general debug info
    println!("\n=== DEBUG INFO ===");
    println!("Target: unknown"); // The TARGET env variable is not set during runtime
    println!(
        "Profile: {}",
        if cfg!(debug_assertions) {
            "debug"
        } else {
            "release"
        }
    );

    // Force a flush to ensure all output is printed
    println!("\n=== ENTERING PANIC LOOP ===");

    // Custom panic handler loop - no automatic reset
    // The system will remain in this loop until manually reset
    loop {
        // Small delay to prevent overwhelming the output
        for _ in 0..1000000 {
            core::hint::spin_loop();
        }
    }
}

use sh8601_rs::{
    ColorMode, DMA_CHUNK_SIZE, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    framebuffer_size,
};

/// A wrapper around a boxed array that implements FrameBufferBackend.
pub struct HeapBuffer<C: PixelColor, const N: usize>(Box<[C; N]>);

impl<C: PixelColor, const N: usize> HeapBuffer<C, N> {
    pub fn new(data: Box<[C; N]>) -> Self {
        Self(data)
    }
}

impl<C: PixelColor, const N: usize> core::ops::Deref for HeapBuffer<C, N> {
    type Target = [C; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<C: PixelColor, const N: usize> core::ops::DerefMut for HeapBuffer<C, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<C: PixelColor, const N: usize> FrameBufferBackend for HeapBuffer<C, N> {
    type Color = C;
    fn set(&mut self, index: usize, color: Self::Color) {
        self.0[index] = color;
    }
    fn get(&self, index: usize) -> Self::Color {
        self.0[index]
    }
    fn nr_elements(&self) -> usize {
        N
    }
}

// Display configuration for Waveshare ESP32-S3-Touch-AMOLED-1.8
const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb888);

// Type alias for the display driver
type DisplayDriver = Sh8601Driver<Ws18AmoledDriver, ResetDriver<I2c<'static, Blocking>>>;

// Conway's Game of Life grid configuration
const GRID_WIDTH: usize = 52; // 368 / 7 ≈ 52
const GRID_HEIGHT: usize = 64; // 448 / 7 ≈ 64

fn update_game_of_life(
    current: &[[u8; GRID_WIDTH]; GRID_HEIGHT],
    next: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT],
) {
    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let mut neighbors = 0;
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0
                        && nx < GRID_WIDTH as i32
                        && ny >= 0
                        && ny < GRID_HEIGHT as i32
                        && current[ny as usize][nx as usize] > 0
                    {
                        neighbors += 1;
                    }
                }
            }
            let cell = current[y][x];
            next[y][x] = match (cell > 0, neighbors) {
                (true, 2) | (true, 3) => cell.saturating_add(1), // stay alive, age
                (false, 3) => 1,                                 // born
                _ => 0,                                          // dead
            };
        }
    }
}

fn randomize_grid(rng: &mut Rng, grid: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            let mut buf = [0u8; 1];
            rng.read(&mut buf);
            // Randomly set cell to 1 (alive) or 0 (dead)
            *cell = if buf[0] & 1 != 0 { 1 } else { 0 };
        }
    }
}

fn age_to_color(age: u8) -> Rgb888 {
    if age == 0 {
        Rgb888::BLACK
    } else {
        let max_age = 10;
        let a = age.min(max_age) as u32;
        let r = ((255 * a) + 5) / max_age as u32;
        let g = ((255 * a) + 5) / max_age as u32;
        let b = 255; // Keep blue channel constant
        Rgb888::new(r as u8, g as u8, b as u8)
    }
}

fn draw_grid<D: DrawTarget<Color = Rgb888>>(
    display: &mut D,
    grid: &[[u8; GRID_WIDTH]; GRID_HEIGHT],
) -> Result<(), D::Error> {
    let border_color = Rgb888::new(230, 230, 230);
    for (y, row) in grid.iter().enumerate() {
        for (x, &age) in row.iter().enumerate() {
            let point = Point::new(x as i32 * 7, y as i32 * 7);
            if age > 0 {
                // Draw a border then fill with color based on age.
                Rectangle::new(point, Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;
                // Draw an inner cell with color according to age.
                Rectangle::new(point + Point::new(1, 1), Size::new(5, 5))
                    .into_styled(PrimitiveStyle::with_fill(age_to_color(age)))
                    .draw(display)?;
            } else {
                // Draw a dead cell as black.
                Rectangle::new(point, Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(Rgb888::BLACK))
                    .draw(display)?;
            }
        }
    }
    Ok(())
}

fn write_generation<D: DrawTarget<Color = Rgb888>>(
    display: &mut D,
    generation: usize,
) -> Result<(), D::Error> {
    let mut num_str = String::<20>::new();
    write!(num_str, "Gen: {generation}").unwrap();
    Text::new(
        num_str.as_str(),
        Point::new(8, 400),
        MonoTextStyle::new(&FONT_10X20, Rgb888::WHITE),
    )
    .draw(display)?;
    Ok(())
}

// --- Bevy ECS Resources ---

// Framebuffer resource for double buffering
const LCD_H_RES: usize = 368;
const LCD_V_RES: usize = 448;
const LCD_BUFFER_SIZE: usize = LCD_H_RES * LCD_V_RES;

type FbBuffer = HeapBuffer<Rgb888, LCD_BUFFER_SIZE>;
type MyFrameBuf = FrameBuf<Rgb888, FbBuffer>;

#[derive(Resource)]
struct FrameBufferResource {
    frame_buf: MyFrameBuf,
}

impl FrameBufferResource {
    fn new() -> Self {
        let fb_data: Box<[Rgb888; LCD_BUFFER_SIZE]> = Box::new([Rgb888::BLACK; LCD_BUFFER_SIZE]);
        let heap_buffer = HeapBuffer::new(fb_data);
        let frame_buf = MyFrameBuf::new(heap_buffer, LCD_H_RES, LCD_V_RES);
        Self { frame_buf }
    }
}

#[derive(Resource)]
struct GameOfLifeResource {
    grid: [[u8; GRID_WIDTH]; GRID_HEIGHT],
    next_grid: [[u8; GRID_WIDTH]; GRID_HEIGHT],
    generation: usize,
}

impl Default for GameOfLifeResource {
    fn default() -> Self {
        Self {
            grid: [[0; GRID_WIDTH]; GRID_HEIGHT],
            next_grid: [[0; GRID_WIDTH]; GRID_HEIGHT],
            generation: 0,
        }
    }
}

#[derive(Resource)]
struct RngResource(Rng);

// Display resource - NonSend because it contains non-thread-safe components
struct DisplayResource {
    display: DisplayDriver,
}

// --- Bevy ECS Systems ---

const RESET_AFTER_GENERATIONS: usize = 300;

fn update_game_of_life_system(
    mut game: ResMut<GameOfLifeResource>,
    mut rng_res: ResMut<RngResource>,
) {
    // Create a temporary copy of the grid to avoid borrowing issues
    let temp_grid = game.grid;
    update_game_of_life(&temp_grid, &mut game.next_grid);

    // Swap the grids by copying instead of using mem::swap to avoid borrowing issues
    let temp = game.grid;
    game.grid = game.next_grid;
    game.next_grid = temp;

    game.generation += 1;

    if game.generation >= RESET_AFTER_GENERATIONS {
        randomize_grid(&mut rng_res.0, &mut game.grid);
        game.generation = 0;
    }
}

fn render_system(
    mut display_res: NonSendMut<DisplayResource>,
    game: Res<GameOfLifeResource>,
    mut fb_res: ResMut<FrameBufferResource>,
) {
    // Clear the framebuffer
    fb_res.frame_buf.clear(Rgb888::BLACK).unwrap();

    // Draw the game grid
    draw_grid(&mut fb_res.frame_buf, &game.grid).unwrap();
    write_generation(&mut fb_res.frame_buf, game.generation).unwrap();

    // Add centered text overlay
    let line1 = "Rust no_std ESP32-S3";
    let line2 = "Bevy ECS 0.16 no_std";
    let line3 = "AMOLED Display";

    // Calculate text positioning - FONT_10X20 is 10 pixels wide
    let line1_width = line1.len() as i32 * 10;
    let line2_width = line2.len() as i32 * 10;
    let line3_width = line3.len() as i32 * 10;

    let x1 = (LCD_H_RES as i32 - line1_width) / 2;
    let x2 = (LCD_H_RES as i32 - line2_width) / 2;
    let x3 = (LCD_H_RES as i32 - line3_width) / 2;

    let y_center = (LCD_V_RES as i32 - 60) / 2; // Updated for 20px font height

    Text::new(
        line1,
        Point::new(x1, y_center),
        MonoTextStyle::new(&FONT_10X20, Rgb888::WHITE),
    )
    .draw(&mut fb_res.frame_buf)
    .unwrap();

    Text::new(
        line2,
        Point::new(x2, y_center + 20),
        MonoTextStyle::new(&FONT_10X20, Rgb888::WHITE),
    )
    .draw(&mut fb_res.frame_buf)
    .unwrap();

    Text::new(
        line3,
        Point::new(x3, y_center + 40),
        MonoTextStyle::new(&FONT_10X20, Rgb888::WHITE),
    )
    .draw(&mut fb_res.frame_buf)
    .unwrap();

    // Draw the framebuffer content directly to the display
    // Clear the display first
    display_res.display.clear(Rgb888::BLACK).ok();

    // Draw each pixel from the framebuffer as a 1x1 rectangle
    for (y, row) in fb_res.frame_buf.data.chunks_exact(LCD_H_RES).enumerate() {
        for (x, &pixel) in row.iter().enumerate() {
            if pixel != Rgb888::BLACK {
                let point = Point::new(x as i32, y as i32);
                Rectangle::new(point, Size::new(1, 1))
                    .into_styled(PrimitiveStyle::with_fill(pixel))
                    .draw(&mut display_res.display)
                    .ok();
            }
        }
    }

    // Flush the display
    display_res.display.flush().ok();
}

#[main]
fn main() -> ! {
    println!("[MAIN] Starting main function");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    println!("[MAIN] Config created");

    let peripherals = esp_hal::init(config);
    println!("[MAIN] Peripherals initialized");

    println!("[MAIN] Starting up...");
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    println!("[MAIN] PSRAM allocator initialized");

    init_logger_from_env();
    println!("[MAIN] Logger initialized");

    let delay = Delay::new();

    info!("Initializing display...");

    // --- DMA Buffers for SPI ---
    #[allow(clippy::manual_div_ceil)]
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // SPI Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40_u32))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sio0(peripherals.GPIO4)
    .with_sio1(peripherals.GPIO5)
    .with_sio2(peripherals.GPIO6)
    .with_sio3(peripherals.GPIO7)
    .with_cs(peripherals.GPIO12)
    .with_sck(peripherals.GPIO11)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    // I2C Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display
    let reset = ResetDriver::<I2c<'_, Blocking>>::new(i2c);

    // Initialize display driver for the Waveshare 1.8" AMOLED display
    let ws_driver = Ws18AmoledDriver::new(lcd_spi);

    // Instantiate and Initialize Display
    println!("Initializing SH8601 Display...");
    let display_res = Sh8601Driver::new_heap::<_, FB_SIZE>(
        ws_driver,
        reset,
        ColorMode::Rgb888,
        DISPLAY_SIZE,
        delay,
    );

    let display = match display_res {
        Ok(d) => {
            println!("Display initialized successfully.");
            d
        }
        Err(e) => {
            println!("Error initializing display: {:?}", e);
            panic!("Failed to initialize display");
        }
    };

    // Initialize RNG
    let mut rng = Rng::new(peripherals.RNG);

    // Initialize game resources
    let mut game = GameOfLifeResource::default();
    randomize_grid(&mut rng, &mut game.grid);

    // Add a glider pattern
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        if *x < GRID_WIDTH && *y < GRID_HEIGHT {
            game.grid[*y][*x] = 1;
        }
    }

    // Create framebuffer resource
    let fb_res = FrameBufferResource::new();

    // Initialize Bevy ECS World
    let mut world = World::default();
    world.insert_resource(game);
    world.insert_resource(RngResource(rng));
    world.insert_resource(fb_res);

    // Insert display as NonSend resource
    world.insert_non_send_resource(DisplayResource { display });

    // Create schedule and add systems
    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    let loop_delay = Delay::new();

    info!("Entering Bevy ECS main loop...");

    loop {
        schedule.run(&mut world);
        loop_delay.delay_millis(50);
    }
}
