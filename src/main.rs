#![no_std]
#![no_main]

use spi_dma_displayinterface::spi_dma_displayinterface;

use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::{ClockControl, CpuClock},
    dma::DmaPriority,
    gdma::Gdma,
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    Delay, Rng, IO,
};

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    prelude::{DrawTarget, Point, RgbColor},
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
    Drawable,
};

use embedded_graphics_framebuf::FrameBuf;

// Define grid size
const WIDTH: usize = 64;
const HEIGHT: usize = 48;

const RESET_AFTER_GENERATIONS: usize = 500;

use core::fmt::Write;

use heapless::String;

fn write_generation<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    generation: usize,
) -> Result<(), D::Error> {
    // Create a String with a fixed capacity of 20 bytes
    let mut num_str = String::<20>::new();
    // Write the generation number into the string
    // unwrap is safe here since we know the number is at most 20 characters
    write!(num_str, "{}", generation).unwrap();
    // Create the text drawable with the generation number
    Text::new(
        num_str.as_str(),
        Point::new(8, 13),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    // Draw the text to the display
    .draw(display)?;

    Ok(())
}

fn randomize_grid(rng: &mut Rng, grid: &mut [[bool; WIDTH]; HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            // Read a single byte from the RNG
            let mut buf = [0u8; 1];
            rng.read(&mut buf).unwrap();

            // Set the cell to be alive or dead based on the random byte
            *cell = buf[0] & 1 != 0;
        }
    }
}

// Apply the Game of Life rules:
// 1. Any live cell with fewer than two live neighbors dies, as if by underpopulation.
// 2. Any live cell with two or three live neighbors lives on to the next generation.
// 3. Any live cell with more than three live neighbors dies, as if by overpopulation.
// 4. Any dead cell with exactly three live neighbors becomes a live cell, as if by reproduction.
fn update_game_of_life(grid: &mut [[bool; WIDTH]; HEIGHT]) {
    let mut new_grid = [[false; WIDTH]; HEIGHT];

    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let alive_neighbors = count_alive_neighbors(x, y, grid);

            new_grid[y][x] = matches!(
                (grid[y][x], alive_neighbors),
                (true, 2) | (true, 3) | (false, 3)
            );
        }
    }

    // Copy the new state back into the original grid
    *grid = new_grid;
}

fn count_alive_neighbors(x: usize, y: usize, grid: &[[bool; WIDTH]; HEIGHT]) -> u8 {
    let mut count = 0;

    for i in 0..3 {
        for j in 0..3 {
            if i == 1 && j == 1 {
                continue; // Skip the current cell itself
            }

            // Calculate the neighbor's coordinates with wrapping
            let neighbor_x = (x + i + WIDTH - 1) % WIDTH;
            let neighbor_y = (y + j + HEIGHT - 1) % HEIGHT;

            // Increase count if the neighbor is alive
            if grid[neighbor_y][neighbor_x] {
                count += 1;
            }
        }
    }

    count
}

fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[bool; WIDTH]; HEIGHT],
) -> Result<(), D::Error> {
    // Define the border color
    let border_color = Rgb565::new(230, 230, 230); // Gray color

    for (y, row) in grid.iter().enumerate() {
        for (x, &cell) in row.iter().enumerate() {
            if cell {
                // Live cell with border
                // Define the size of the cells and borders
                let cell_size = Size::new(5, 5);
                let border_size = Size::new(7, 7); // Slightly larger for the border

                // Draw the border rectangle
                Rectangle::new(Point::new(x as i32 * 7, y as i32 * 7), border_size)
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;

                // Draw the inner cell rectangle (white)
                Rectangle::new(Point::new(x as i32 * 7 + 1, y as i32 * 7 + 1), cell_size)
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                    .draw(display)?;
            } else {
                // Dead cell without border (black)
                Rectangle::new(Point::new(x as i32 * 7, y as i32 * 7), Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
            }
        }
    }
    Ok(())
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    // let clocks = ClockControl::max(system.clock_control).freeze();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();
    let mut delay = Delay::new(&clocks);

    println!("About to initialize the SPI LED driver");
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sclk = io.pins.gpio7;
    let mosi = io.pins.gpio6;
    let cs = io.pins.gpio5;
    let miso = io.pins.gpio2;
    // let sda = io.pins.gpio8;
    // let scl = io.pins.gpio18;
    let dc = io.pins.gpio4.into_push_pull_output();
    let mut backlight = io.pins.gpio45.into_push_pull_output();
    let reset = io.pins.gpio48.into_push_pull_output();

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let spi = Spi::new(
        peripherals.SPI2,
        sclk,
        mosi,
        miso,
        cs,
        60u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    )
    .with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    println!("SPI ready");

    let di = spi_dma_displayinterface::new_no_cs(320 * 240 *2, spi, dc);

    // ESP32-S3-BOX display initialization workaround: Wait for the display to power up.
    // If delay is 250ms, picture will be fuzzy.
    // If there is no delay, display is blank
    delay.delay_ms(500u32);

    let mut display = match mipidsi::Builder::ili9342c_rgb565(di)
        .with_display_size(320, 240)
        .with_orientation(mipidsi::Orientation::PortraitInverted(false))
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .init(&mut delay, Some(reset))
    {
        Ok(display) => display,
        Err(_e) => {
            // Handle the error and possibly exit the application
            panic!("Display initialization failed");
        }
    };

    let _ = backlight.set_high();

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello Conway!");

    let mut grid: [[bool; WIDTH]; HEIGHT] = [[false; WIDTH]; HEIGHT];
    let mut rng = Rng::new(peripherals.RNG);
    randomize_grid(&mut rng, &mut grid);
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        grid[*y][*x] = true;
    }
    let mut generation_count = 0;

    let mut data = [Rgb565::BLACK; 320 * 240];
    let mut fbuf = FrameBuf::new(&mut data, 320, 240);

    loop {
        // Update the game state
        update_game_of_life(&mut grid);

        // Draw the updated grid on the display
        draw_grid(&mut fbuf, &grid).unwrap();

        generation_count += 1;

        if generation_count >= RESET_AFTER_GENERATIONS {
            randomize_grid(&mut rng, &mut grid);
            generation_count = 0; // Reset the generation counter
        }

        write_generation(&mut fbuf, generation_count).unwrap();

        let pixel_iterator = fbuf.into_iter().map(|p| p.1);
        let _ = display.set_pixels(0, 0, 319, 240, pixel_iterator);

        // Add a delay to control the simulation speed
        delay.delay_ms(100u32);
    }
}
