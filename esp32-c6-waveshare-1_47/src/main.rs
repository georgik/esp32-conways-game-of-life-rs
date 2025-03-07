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

// Physical display resolution.
const LCD_H_RES: usize = 206;
const LCD_V_RES: usize = 320;

// -----------------------------------------------------------------------------
// Line Buffer (Partial Update) Definitions
// -----------------------------------------------------------------------------

// We choose a chunk height (number of lines) for partial updates.
const LINE_BUFFER_LINES: usize = 10;
const LINE_BUFFER_SIZE: usize = LCD_H_RES * LINE_BUFFER_LINES;

// This resource holds a line buffer used to update a few lines at a time.
#[derive(Resource)]
struct LineBufferResource {
    buffer: [Rgb565; LINE_BUFFER_SIZE],
}

impl LineBufferResource {
    fn new() -> Self {
        Self {
            buffer: [Rgb565::BLACK; LINE_BUFFER_SIZE],
        }
    }
}

// -----------------------------------------------------------------------------
// Game of Life Definitions (unchanged)
// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// Drawing Functions for Partial Update
// -----------------------------------------------------------------------------

/// Draw the grid cells that fall within the current vertical chunk.
/// `y_offset` is the starting y coordinate (in pixels) of the chunk.
/// `chunk_height` is the number of lines in the chunk.
fn draw_grid_partial<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[bool; GRID_WIDTH]; GRID_HEIGHT],
    y_offset: usize,
    chunk_height: usize,
) -> Result<(), D::Error> {
    let border_color = Rgb565::new(230, 230, 230);
    // Our cells are drawn at multiples of 7 pixels.
    for (gy, row) in grid.iter().enumerate() {
        let cell_y = gy * 7;
        let cell_bottom = cell_y + 7;
        // Skip cells that are entirely above or below the chunk.
        if cell_bottom <= y_offset || cell_y >= y_offset + chunk_height {
            continue;
        }
        for (gx, &cell) in row.iter().enumerate() {
            let cell_x = gx * 7;
            // Draw the cell relative to the chunk (subtract y_offset).
            let draw_point = Point::new(cell_x as i32, cell_y as i32 - y_offset as i32);
            if cell {
                Rectangle::new(draw_point, Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;
                Rectangle::new(draw_point + Point::new(1, 1), Size::new(5, 5))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                    .draw(display)?;
            } else {
                Rectangle::new(draw_point, Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
            }
        }
    }
    Ok(())
}

/// Draw the generation text only in the top chunk.
fn write_generation_partial<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    generation: usize,
    y_offset: usize,
) -> Result<(), D::Error> {
    if y_offset == 0 {
        let mut num_str = heapless::String::<20>::new();
        write!(num_str, "{}", generation).unwrap();
        Text::new(
            num_str.as_str(),
            Point::new(8, 13),
            MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
        )
        .draw(display)?;
    }
    Ok(())
}

// -----------------------------------------------------------------------------
// ECS Resources and Systems (unchanged except for using NonSend for Display)
// -----------------------------------------------------------------------------

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

// We cannot mark MyDisplay as Sync because it contains raw pointers from DMA,
// so we wrap it as a NonSend resource.
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

/// Render system: update the display in partial (line buffer) mode.
fn render_system(
    mut display_res: NonSendMut<DisplayResource>,
    game: Res<GameOfLifeResource>,
    mut lb_res: ResMut<LineBufferResource>,
) {
    // For each chunk of LINE_BUFFER_LINES pixels in the vertical direction:
    for y_offset in (0..LCD_V_RES).step_by(LINE_BUFFER_LINES) {
        // Determine the height of this chunk (it may be smaller at the bottom)
        let chunk_height = if y_offset + LINE_BUFFER_LINES > LCD_V_RES {
            LCD_V_RES - y_offset
        } else {
            LINE_BUFFER_LINES
        };

        // Create a temporary FrameBuf over our line buffer.
        // Our line buffer (lb_res.buffer) is used as the backend.
        let mut chunk_fbuf = FrameBuf::new(&mut lb_res.buffer, LCD_H_RES, chunk_height);

        // Clear the chunk.
        chunk_fbuf.clear(Rgb565::BLACK).unwrap();
        // Draw the portion of the game grid that intersects this chunk.
        draw_grid_partial(&mut chunk_fbuf, &game.grid, y_offset, chunk_height).unwrap();
        // Write generation text if applicable.
        write_generation_partial(&mut chunk_fbuf, game.generation, y_offset).unwrap();

        // Define the area on the physical display corresponding to this chunk.
        let area = Rectangle::new(Point::new(0, y_offset as i32), Size::new(LCD_H_RES as u32, chunk_height as u32));
        // Flush this chunk to the display.
        display_res
            .display
            .fill_contiguous(&area, chunk_fbuf.data.iter().copied())
            .unwrap();
    }
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // With a reduced line buffer, adjust heap size if necessary.
    esp_alloc::heap_allocator!(size: 140 * 1024);
    init_logger_from_env();

    // --- DMA Buffers for SPI ---
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
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
    // Leak a Box to obtain a 'static mutable buffer for SPI transfers.
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

    // Create the line buffer resource.
    let lb_res = LineBufferResource::new();

    let mut world = World::default();
    world.insert_resource(game);
    world.insert_resource(RngResource(rng_instance));
    // Insert the display as a non-send resource because its DMA descriptors aren’t Sync.
    world.insert_non_send_resource(DisplayResource { display });
    world.insert_resource(lb_res);

    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    let mut loop_delay = Delay::new();
    loop {
        schedule.run(&mut world);
        loop_delay.delay_ms(100u32);
    }
}
