#![no_std]
#![no_main]

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
use esp_hal::delay::Delay;
use esp_hal::{
    gpio::{DriveMode, Level, Output, OutputConfig},
    rng::Rng,
    spi::master::Spi,
    main,
    time::Rate,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_println::logger::init_logger_from_env;
use log::info;
use mipidsi::interface::SpiInterface;
use mipidsi::{models::ILI9486Rgb565, Builder};
// Bevy ECS (no_std) imports:
use bevy_ecs::prelude::*;

// --- Game of Life Definitions ---

// Grid dimensions and generation reset threshold.
const WIDTH: usize = 64;
const HEIGHT: usize = 48;
const RESET_AFTER_GENERATIONS: usize = 500;

// Randomize the grid using the provided RNG.
fn randomize_grid(rng: &mut Rng, grid: &mut [[bool; WIDTH]; HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            let mut buf = [0u8; 1];
            rng.read(&mut buf);
            *cell = buf[0] & 1 != 0;
        }
    }
}

// Standard Conway’s Game of Life update.
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
    *grid = new_grid;
}

fn count_alive_neighbors(x: usize, y: usize, grid: &[[bool; WIDTH]; HEIGHT]) -> u8 {
    let mut count = 0;
    for i in 0..3 {
        for j in 0..3 {
            if i == 1 && j == 1 { continue; }
            let neighbor_x = (x + i + WIDTH - 1) % WIDTH;
            let neighbor_y = (y + j + HEIGHT - 1) % HEIGHT;
            if grid[neighbor_y][neighbor_x] {
                count += 1;
            }
        }
    }
    count
}

// Draw the grid to the display.
fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[bool; WIDTH]; HEIGHT],
) -> Result<(), D::Error> {
    let border_color = Rgb565::new(230, 230, 230);
    for (y, row) in grid.iter().enumerate() {
        for (x, &cell) in row.iter().enumerate() {
            if cell {
                // Live cell: draw a border and an inner white cell.
                let cell_size = Size::new(5, 5);
                let border_size = Size::new(7, 7);
                Rectangle::new(Point::new(x as i32 * 7, y as i32 * 7), border_size)
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;
                Rectangle::new(Point::new(x as i32 * 7 + 1, y as i32 * 7 + 1), cell_size)
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::WHITE))
                    .draw(display)?;
            } else {
                // Dead cell: fill with black.
                Rectangle::new(Point::new(x as i32 * 7, y as i32 * 7), Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
            }
        }
    }
    Ok(())
}

// Write the generation number to the display.
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

// Resource holding the Game of Life state.
#[derive(Default)]
struct GameOfLifeResource {
    grid: [[bool; WIDTH]; HEIGHT],
    generation: usize,
}

// Resource wrapping the RNG.
struct RngResource(Rng);

// Resource for the display. The type parameter D must implement DrawTarget<Color = Rgb565>.
struct DisplayResource<D: DrawTarget<Color = Rgb565>> {
    display: D,
}

// System to update the Game of Life state.
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

// System to render the current game state to the display.
fn render_system<D: DrawTarget<Color = Rgb565>>(
    mut display_res: ResMut<DisplayResource<D>>,
    game: Res<GameOfLifeResource>,
) {
    // Clear the display.
    display_res.display.clear(Rgb565::BLACK).unwrap();
    // Draw the grid.
    draw_grid(&mut display_res.display, &game.grid).unwrap();
    // Write the generation number.
    write_generation(&mut display_res.display, game.generation).unwrap();
}

// --- Main Function ---

#[main]
fn main() -> ! {
    // Initialize peripherals and logger.
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_logger_from_env();

    // --- Display Setup ---
    // Set up SPI for the display.
    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(esp_hal::spi::Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO7)
    .with_mosi(peripherals.GPIO6);
    // Chip select output.
    let cs_output = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs_output).unwrap();
    let lcd_dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let mut buffer = [0_u8; 512];
    let di = SpiInterface::new(spi_device, lcd_dc, &mut buffer);

    let mut delay = Delay::new();
    delay.delay_ns(500_000u32);

    // Reset pin for the display.
    let reset = Output::new(
        peripherals.GPIO48,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );
    // Initialize the display using mipidsi's builder.
    let display = Builder::new(ILI9486Rgb565, di)
        .reset_pin(reset)
        .init(&mut delay)
        .unwrap();

    // Turn on the backlight.
    let mut backlight =
        Output::new(peripherals.GPIO47, Level::High, OutputConfig::default());
    backlight.set_high();

    info!("Hello Conway with Bevy ECS!");

    // --- Initialize Game Resources ---
    // Create the game state resource.
    let mut game = GameOfLifeResource::default();
    {
        // Use RNG to randomize the grid.
        let mut rng = Rng::new(peripherals.RNG);
        randomize_grid(&mut rng, &mut game.grid);
        // Optionally, add a glider pattern.
        let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
        for (x, y) in glider.iter() {
            game.grid[*y][*x] = true;
        }
    }

    // Build the ECS world and insert resources.
    let mut world = World::default();
    world.insert_resource(game);
    world.insert_resource(RngResource(Rng::new(peripherals.RNG)));
    world.insert_resource(DisplayResource { display });

    // Create a schedule and add our systems.
    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system::<_>);

    // Main loop: run the ECS schedule and delay.
    loop {
        schedule.run(&mut world);
        delay.delay_ms(100u32);
    }
}
