#![no_std]
#![no_main]

// ESP-IDF App Descriptor required by newer espflash
esp_bootloader_esp_idf::esp_app_desc!();

// use esp_bsp::prelude::*;
// use esp_display_interface_spi_dma::display_interface_spi_dma;
use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    prelude::{DrawTarget, Point, RgbColor},
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
    Drawable,
};
use embedded_hal_bus::spi::ExclusiveDevice;
#[allow(unused_imports)]
use esp_backtrace as _;
// use esp_hal::gpio::OutputOpenDrain;
use esp_hal::rng::Rng;
use esp_hal::{
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig},
    main,
    spi::master::Spi,
    time::Rate,
};
use mipidsi::interface::SpiInterface;
// use embedded_graphics_framebuf::FrameBuf;
use embedded_hal::delay::DelayNs;
use log::info;

// Define grid size
const WIDTH: usize = 64;
const HEIGHT: usize = 48;

const RESET_AFTER_GENERATIONS: usize = 500;

use core::fmt::Write;
// use esp_hal::dma::Owner::Dma;
use esp_hal::spi::Mode;
use heapless::String;
use mipidsi::{models::ILI9486Rgb565, Builder};

fn write_generation<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    generation: usize,
) -> Result<(), D::Error> {
    // Create a String with a fixed capacity of 20 bytes
    let mut num_str = String::<20>::new();
    // Write the generation number into the string
    // unwrap is safe here since we know the number is at most 20 characters
    write!(num_str, "{generation}").unwrap();
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
            rng.read(&mut buf);

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

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    // let spi = lcd_spi!(peripherals);
    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    )
        .unwrap()
        .with_sck(peripherals.GPIO7)
        .with_mosi(peripherals.GPIO6)
        // .with_cs((peripherals.GPIO5))
    ;
    // .with_dma((peripherals.DMA_CH0));
    // let mut spi = Spi::new(
    //     peripherals.SPI2,
    //     esp_hal::spi::master::Config::default()
    //         .with_frequency(100.kHz())
    //         .with_mode(Mode::_0),
    // )
    //     .unwrap()
    //     .with_sck(sclk)
    //     .with_miso(miso)
    //     .with_mosi(mosi)
    //     .with_cs(cs)
    //     .with_dma(peripherals.DMA_CH0);

    // let di = lcd_display_interface!(peripherals, spi);
    // let lcd_dc = Output::new($dc_pin, Level::Low);
    let mut buffer = [0_u8; 512];

    let lcd_dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    // Define the display interface with no chip select
    let cs_output = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs_output).unwrap();
    let di = SpiInterface::new(spi_device, lcd_dc, &mut buffer);

    // let di = display_interface_spi_dma::new_no_cs(crate::LCD_MEMORY_SIZE, spi, lcd_dc);
    // let di = SpiInterface::new(spi_device, dc, &mut buffer);
    // }
    let mut delay = Delay::new();
    delay.delay_ns(500_000u32);

    // let mut display = lcd_display!(peripherals, di).init(&mut delay).unwrap();
    // let mut display =     mipidsi::Builder::new(mipidsi::models::ILI9342CRgb565, di)
    //     .display_size((320 as u16), (240 as u16))
    //     .orientation((mipidsi::options::Orientation::new()
    //         .flip_vertical()
    //         .flip_horizontal()
    //     ))
    //     .color_order(mipidsi::options::ColorOrder::Bgr)
    //     .reset_pin(OutputOpenDrain::new(peripherals.GPIO48, Level::High, Pull::Up)
    //     );

    let reset = Output::new(
        peripherals.GPIO48,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );

    let mut display = Builder::new(ILI9486Rgb565, di)
        .reset_pin(reset)
        .init(&mut delay)
        .unwrap();

    // Use the `lcd_backlight_init` macro to turn on the backlight
    let mut backlight = Output::new(peripherals.GPIO47, Level::High, OutputConfig::default());
    backlight.set_high();
    // lcd_backlight_init!(peripherals);

    info!("Hello Conway!");

    let mut grid: [[bool; WIDTH]; HEIGHT] = [[false; WIDTH]; HEIGHT];
    let mut rng = Rng::new(peripherals.RNG);
    randomize_grid(&mut rng, &mut grid);
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        grid[*y][*x] = true;
    }
    let mut generation_count = 0;

    // let mut data = [Rgb565::BLACK; 320 * 240];
    // let mut fbuf = FrameBuf::new(&mut data, 320, 240);
    display.clear(Rgb565::BLACK).unwrap();

    loop {
        // Update the game state
        update_game_of_life(&mut grid);

        // Draw the updated grid on the display
        // draw_grid(&mut fbuf, &grid).unwrap();
        draw_grid(&mut display, &grid).unwrap();

        generation_count += 1;

        if generation_count >= RESET_AFTER_GENERATIONS {
            randomize_grid(&mut rng, &mut grid);
            generation_count = 0; // Reset the generation counter
        }

        // write_generation(&mut fbuf, generation_count).unwrap();
        write_generation(&mut display, generation_count).unwrap();

        // let pixel_iterator = fbuf.into_iter().map(|p| p.1);
        // // let _ = display.set_pixels(0, 0, 319, 240, pixel_iterator);
        // use mipidsi::interface::InterfacePixelFormat;

        // let pixel_iterator = fbuf.into_iter().map(|p| p.1);

        // // Send the pixels to the display
        // Rgb565::send_pixels(&mut display, pixel_iterator).unwrap();

        // Add a delay to control the simulation speed
        delay.delay_ms(100u32);
    }
}
