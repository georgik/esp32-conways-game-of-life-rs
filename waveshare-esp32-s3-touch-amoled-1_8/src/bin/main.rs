#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_graphics::mono_font::{ascii::FONT_8X13, MonoTextStyle};
use embedded_graphics::text::Text;
use heapless::String;

use alloc::boxed::Box;
use embedded_graphics::{
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    Drawable,
};
use embedded_graphics_framebuf::backends::FrameBufferBackend;
use embedded_graphics_framebuf::FrameBuf;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Ticker;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::{timg::TimerGroup, AnyTimer};
use esp_hal_embassy::Executor;
use log::{error, info};
use static_cell::StaticCell;

use esp_hal::rng::Rng;
use esp_hal::system::{CpuControl, Stack};
use esp_println::logger::init_logger_from_env;
use esp_println::println;
use esp_backtrace as _;

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE,
};

// esp-backtrace handles panic, so we don't need a custom panic handler

extern crate alloc;

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
        &*self.0
    }
}

impl<C: PixelColor, const N: usize> core::ops::DerefMut for HeapBuffer<C, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut *self.0
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
type DisplayDriver = Sh8601Driver<Ws18AmoledDriver<esp_hal::spi::master::Spi<'static, esp_hal::spi::SPI2, esp_hal::dma::DmaChannel0>>, ResetDriver<esp_hal::i2c::master::I2c<'static, esp_hal::i2c::I2C0>>>;

// Conway's Game of Life grid configuration
const GRID_WIDTH: usize = 52;  // 368 / 7 ≈ 52
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
                    if nx >= 0 && nx < GRID_WIDTH as i32 && ny >= 0 && ny < GRID_HEIGHT as i32 {
                        if current[ny as usize][nx as usize] > 0 {
                            neighbors += 1;
                        }
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
    write!(num_str, "Gen: {}", generation).unwrap();
    Text::new(
        num_str.as_str(),
        Point::new(8, 400),
        MonoTextStyle::new(&FONT_8X13, Rgb888::WHITE),
    )
    .draw(display)?;
    Ok(())
}

// Embassy multicore: allocate app core stack
static mut APP_CORE_STACK: Stack<8192> = Stack::new();
static DISPLAY_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    // Initialize TimerGroup and timer1 for app core Embassy time driver
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let timer1: AnyTimer = timg1.timer0.into();
    
    println!("Starting up...");
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    println!("PSRAM allocator initialized");

    init_logger_from_env();

    let delay = Delay::new();

    info!("Initializing display...");

    // --- DMA Buffers for SPI ---
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
    let reset = ResetDriver::new(i2c);

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
            loop {}
        }
    };

    // Leak the display to get a static reference
    let display_ref = Box::leak(Box::new(display));

    // Prepare RNG for app core task
    let rng_for_app = Rng::new(peripherals.RNG);

    info!("Entering main loop...");

    // Signal to app core that display is ready
    DISPLAY_READY.signal(());
    
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();

    // Spawn Conway task on app core (core 1)
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let _app_core = cpu_control.start_app_core(unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) }, move || {
        // Initialize Embassy time driver on app core
        esp_hal_embassy::init([timer0, timer1]);
        info!("[CORE 1] App core started, initializing Conway task");
        
        // Initialize and run Embassy executor on app core
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner
                .spawn(conway_task(display_ref, rng_for_app))
                .ok();
        });
    });

    // Core 0: Keep main thread alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

// Conway update task running on core 1
#[embassy_executor::task]
async fn conway_task(display: &'static mut DisplayDriver, mut rng: Rng) {
    // Wait until display is ready
    loop {
        if DISPLAY_READY.try_take().is_some() {
            break;
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
    }

    let mut game_grid = Box::new([[0u8; GRID_WIDTH]; GRID_HEIGHT]);
    randomize_grid(&mut rng, &mut *game_grid);
    let mut next_grid = Box::new([[0u8; GRID_WIDTH]; GRID_HEIGHT]);

    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(150));
    let mut generation_count: usize = 0;
    const RESET_AFTER_GENERATIONS: usize = 300;
    
    loop {
        // Clear display
        display.clear(Rgb888::BLACK).ok();
        
        // Update game of life
        update_game_of_life(&*game_grid, &mut *next_grid);
        core::mem::swap(&mut game_grid, &mut next_grid);
        
        // Draw grid
        draw_grid(display, &*game_grid).ok();
        generation_count += 1;
        write_generation(display, generation_count).ok();
        
        // Flush display
        if let Err(e) = display.flush() {
            error!("Error flushing display: {:?}", e);
        }
        
        // Reset after a certain number of generations
        if generation_count >= RESET_AFTER_GENERATIONS {
            randomize_grid(&mut rng, &mut *game_grid);
            generation_count = 0;
        }
        
        ticker.next().await;
    }
}
