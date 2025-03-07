#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;

use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
    Drawable,
};
use embedded_graphics_framebuf::FrameBuf;
use esp_hal::delay::Delay;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    rng::Rng,
    spi::master::{Spi, SpiDmaBus},
    Blocking,
    main,
    time::Rate,
};
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_println::{logger::init_logger_from_env, println};
use log::info;
use mipidsi::{interface::SpiInterface, options::ColorInversion};
use mipidsi::{models::ST7789, Builder};
use bevy_ecs::prelude::*; // includes NonSend and NonSendMut

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("Panic: {}", _info);
    loop {}
}

// --- Type Alias for the Concrete Display ---
// Use the DMA-enabled SPI bus type.
type MyDisplay = mipidsi::Display<
    SpiInterface<
        'static,
        ExclusiveDevice<SpiDmaBus<'static, Blocking>, Output<'static>, Delay>,
        Output<'static>
    >,
    ST7789,
    Output<'static>
>;

// --- LCD Resolution and FrameBuffer Type Aliases ---
const LCD_H_RES: usize = 206;
const LCD_V_RES: usize = 320;
const LCD_BUFFER_SIZE: usize = LCD_H_RES * LCD_V_RES;

// We want our pixels stored as Rgb565.
type FbBuffer = [Rgb565; LCD_BUFFER_SIZE];

// Define a type alias for the complete FrameBuf:
// First parameter: Pixel color type, second: Owned backend.
type MyFrameBuf = FrameBuf<Rgb565, FbBuffer>;

#[derive(Resource)]
struct FrameBufferResource {
    frame_buf: MyFrameBuf,
}

impl FrameBufferResource {
    fn new() -> Self {
        // Allocate the framebuffer data as an owned array of Rgb565.
        let fb_data: FbBuffer = *Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
        let frame_buf = MyFrameBuf::new(fb_data, LCD_H_RES, LCD_V_RES);
        Self { frame_buf }
    }
}

// --- Game of Life Definitions ---
const GRID_WIDTH: usize = 64;
const GRID_HEIGHT: usize = 48;
const RESET_AFTER_GENERATIONS: usize = 500;

fn randomize_grid(rng: &mut Rng, grid: &mut [[bool; GRID_WIDTH]; GRID_HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            let mut buf = [0u8; 1];
            rng.read(&mut buf);
            *cell = buf[0] & 1 != 0;
        }
    }
}

fn update_game_of_life(grid: &mut [[bool; GRID_WIDTH]; GRID_HEIGHT]) {
    let mut new_grid = [[false; GRID_WIDTH]; GRID_HEIGHT];
    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let alive_neighbors = count_alive_neighbors(x, y, grid);
            new_grid[y][x] = matches!(
                (grid[y][x], alive_neighbors),
                (true, 2) | (true, 3) | (false, 3)
            );
        }
    }
    *grid = new_grid;
}

fn count_alive_neighbors(x: usize, y: usize, grid: &[[bool; GRID_WIDTH]; GRID_HEIGHT]) -> u8 {
    let mut count = 0;
    for i in 0..3 {
        for j in 0..3 {
            if i == 1 && j == 1 { continue; }
            let nx = (x + i + GRID_WIDTH - 1) % GRID_WIDTH;
            let ny = (y + j + GRID_HEIGHT - 1) % GRID_HEIGHT;
            if grid[ny][nx] {
                count += 1;
            }
        }
    }
    count
}

fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[bool; GRID_WIDTH]; GRID_HEIGHT],
) -> Result<(), D::Error> {
    let border_color = Rgb565::new(230, 230, 230);
    for (y, row) in grid.iter().enumerate() {
        for (x, &cell) in row.iter().enumerate() {
            if cell {
                let cell_size = Size::new(5, 5);
                let border_size = Size::new(7, 7);
                Rectangle::new(Point::new(x as i32 * 7, y as i32 * 7), border_size)
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;
                Rectangle::new(Point::new(x as i32 * 7 + 1, y as i32 * 7 + 1), cell_size)
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                    .draw(display)?;
            } else {
                Rectangle::new(Point::new(x as i32 * 7, y as i32 * 7), Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
            }
        }
    }
    Ok(())
}

fn write_generation<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    generation: usize,
) -> Result<(), D::Error> {
    let mut num_str = heapless::String::<20>::new();
    write!(num_str, "{}", generation).unwrap();
    Text::new(
        num_str.as_str(),
        Point::new(8, 13),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    .draw(display)?;
    Ok(())
}

// --- ECS Resources and Systems ---

#[derive(Resource)]
struct GameOfLifeResource {
    grid: [[bool; GRID_WIDTH]; GRID_HEIGHT],
    generation: usize,
}

impl Default for GameOfLifeResource {
    fn default() -> Self {
        Self {
            grid: [[false; GRID_WIDTH]; GRID_HEIGHT],
            generation: 0,
        }
    }
}

#[derive(Resource)]
struct RngResource(Rng);

// Because our display type contains DMA descriptors and raw pointers, it isn’t Sync.
// We wrap it in a NonSend resource so that Bevy doesn’t require Sync.
struct DisplayResource {
    display: MyDisplay,
}

fn update_game_of_life_system(
    mut game: ResMut<GameOfLifeResource>,
    mut rng_res: ResMut<RngResource>,
) {
    update_game_of_life(&mut game.grid);
    game.generation += 1;
    if game.generation >= RESET_AFTER_GENERATIONS {
        randomize_grid(&mut rng_res.0, &mut game.grid);
        game.generation = 0;
    }
}

/// Render the game state by drawing into the offscreen framebuffer and then flushing
/// it to the display via DMA. Instead of converting to a byte slice, we simply iterate
/// over the owned framebuffer.
fn render_system(
    mut display_res: NonSendMut<DisplayResource>,
    game: Res<GameOfLifeResource>,
    mut fb_res: ResMut<FrameBufferResource>,
) {
    // Clear the framebuffer.
    fb_res.frame_buf.clear(Rgb565::BLACK).unwrap();
    // Draw the game grid and generation into the framebuffer.
    draw_grid(&mut fb_res.frame_buf, &game.grid).unwrap();
    write_generation(&mut fb_res.frame_buf, game.generation).unwrap();

    // Define the area covering the entire framebuffer.
    let area = Rectangle::new(Point::zero(), fb_res.frame_buf.size());
    // Since our backend is an owned array of Rgb565, we can simply iterate over it.
    display_res
        .display
        .fill_contiguous(&area, fb_res.frame_buf.data.iter().copied())
        .unwrap();
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 150 * 1024);
    init_logger_from_env();

    // --- DMA Buffers for SPI ---
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(8912);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // --- Display Setup using BSP values ---
    // SPI: SCK = GPIO7, MOSI = GPIO6, CS = GPIO14.
    let spi = Spi::<Blocking>::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO7)
    .with_mosi(peripherals.GPIO6)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf);
    let cs_output = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    let spi_delay = Delay::new();
    let spi_device = ExclusiveDevice::new(spi, cs_output, spi_delay).unwrap();

    // LCD interface: DC = GPIO15.
    let lcd_dc = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    // Leak a Box to obtain a 'static mutable buffer.
    let buffer: &'static mut [u8; 512] = Box::leak(Box::new([0_u8; 512]));
    let di = SpiInterface::new(spi_device, lcd_dc, buffer);

    let mut display_delay = Delay::new();
    display_delay.delay_ns(500_000u32);

    // Reset pin: GPIO21 (active low per BSP).
    let reset = Output::new(
        peripherals.GPIO21,
        Level::Low,
        OutputConfig::default(),
    );
    // Initialize the display using mipidsi's builder.
    let mut display: MyDisplay = Builder::new(ST7789, di)
        .reset_pin(reset)
        .display_size(206, 320)
        .invert_colors(ColorInversion::Inverted)
        .init(&mut display_delay)
        .unwrap();

    display.clear(Rgb565::BLUE).unwrap();

    // Backlight on GPIO22.
    let mut backlight = Output::new(peripherals.GPIO22, Level::Low, OutputConfig::default());
    backlight.set_high();

    info!("Display initialized");

    // --- Initialize Game Resources ---
    let mut game = GameOfLifeResource::default();
    let mut rng_instance = Rng::new(peripherals.RNG);
    randomize_grid(&mut rng_instance, &mut game.grid);
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        game.grid[*y][*x] = true;
    }

    // Create the framebuffer resource.
    let fb_res = FrameBufferResource::new();

    let mut world = World::default();
    world.insert_resource(game);
    world.insert_resource(RngResource(rng_instance));
    // The DisplayResource is non‑send because of DMA pointers, so keep that as non‑send.
    world.insert_non_send_resource(DisplayResource { display });
    // Insert the framebuffer resource as a normal resource.
    world.insert_resource(fb_res);


    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    let mut loop_delay = Delay::new();

    loop {
        schedule.run(&mut world);
        loop_delay.delay_ms(50u32);
    }
}
