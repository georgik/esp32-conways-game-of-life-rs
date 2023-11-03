#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay,
    IO,
    spi::{master::Spi, SpiMode},
    Rng,
};

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    prelude::{DrawTarget, Point, RgbColor},
    text::Text,
    Drawable,
    pixelcolor::Rgb565,
    primitives::{PrimitiveStyle, Rectangle},
    prelude::*,
};

// Define grid size
const WIDTH: usize = 64;
const HEIGHT: usize = 48;

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

fn update_game_of_life(grid: &mut [[bool; WIDTH]; HEIGHT]) {
    let mut new_grid = [[false; WIDTH]; HEIGHT];

    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let alive_neighbors = count_alive_neighbors(x, y, &grid);

            // Apply the Game of Life rules:
            // 1. Any live cell with fewer than two live neighbors dies, as if by underpopulation.
            // 2. Any live cell with two or three live neighbors lives on to the next generation.
            // 3. Any live cell with more than three live neighbors dies, as if by overpopulation.
            // 4. Any dead cell with exactly three live neighbors becomes a live cell, as if by reproduction.
            new_grid[y][x] = match (grid[y][x], alive_neighbors) {
                (true, 2) | (true, 3) => true,
                (false, 3) => true,
                _ => false,
            };
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
    for (y, row) in grid.iter().enumerate() {
        for (x, &cell) in row.iter().enumerate() {
            let color = if cell { Rgb565::WHITE } else { Rgb565::BLACK };
            Rectangle::new(Point::new(x as i32 * 5, y as i32 * 5), Size::new(5, 5))
                .into_styled(PrimitiveStyle::with_fill(color))
                .draw(display)?;
        }
    }
    Ok(())
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    println!("About to initialize the SPI LED driver");
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sclk = io.pins.gpio7;
    let mosi = io.pins.gpio6;
    // let sda = io.pins.gpio8;
    // let scl = io.pins.gpio18;
    let dc = io.pins.gpio4.into_push_pull_output();
    let mut backlight = io.pins.gpio45.into_push_pull_output();
    let reset = io.pins.gpio48.into_push_pull_output();

    println!("SPI LED driver initialized");
    let spi = Spi::new_no_cs_no_miso(
        peripherals.SPI2,
        sclk,
        mosi,
        60u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    );

    println!("SPI ready");

    let di = SPIInterfaceNoCS::new(spi, dc);

    // ESP32-S3-BOX display initialization workaround: Wait for the display to power up.
    // If delay is 250ms, picture will be fuzzy.
    // If there is no delay, display is blank
    delay.delay_ms(500u32);

    let mut display = match mipidsi::Builder::ili9342c_rgb565(di)
        .with_display_size(320, 240)
        .with_orientation(mipidsi::Orientation::PortraitInverted(false))
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .init(&mut delay, Some(reset)) {
        Ok(display) => display,
        Err(e) => {
            // Handle the error and possibly exit the application
            panic!("Display initialization failed");
        }
    };

    backlight.set_high();

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

    loop {
        // Update the game state
        update_game_of_life(&mut grid);

        // Draw the updated grid on the display
        draw_grid(&mut display, &grid).unwrap();

        // Refresh the display here if required by your display driver
        // display.flush().unwrap();

        // Add a delay to control the simulation speed
        delay.delay_ms(100u32);
    }
}
