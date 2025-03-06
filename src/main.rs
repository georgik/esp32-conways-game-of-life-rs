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
use esp_hal::delay::Delay;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    rng::Rng,
    spi::master::Spi,
    Blocking,
    main,
    time::Rate,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_println::{logger::init_logger_from_env, println};
use log::info;
use mipidsi::{interface::SpiInterface, options::ColorInversion};

use mipidsi::{models::ST7789, Builder};
// Bevy ECS (no_std) imports:
use bevy_ecs::prelude::*;

// --- Type Alias for the Concrete Display ---
// Now we specify that the SPI interface uses Delay.
type MyDisplay = mipidsi::Display<
    SpiInterface<
        'static,
        ExclusiveDevice<Spi<'static, Blocking>, Output<'static>, Delay>,
        Output<'static>
    >,
    ST7789,
    Output<'static>
>;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("Panic: {}", _info);
    loop {}
}

#[allow(unused_imports)]
use esp_alloc::EspHeap;

// --- Game of Life Definitions ---

const WIDTH: usize = 64;
const HEIGHT: usize = 48;
const RESET_AFTER_GENERATIONS: usize = 500;

fn randomize_grid(rng: &mut Rng, grid: &mut [[bool; WIDTH]; HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            let mut buf = [0u8; 1];
            rng.read(&mut buf);
            *cell = buf[0] & 1 != 0;
        }
    }
}

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

fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[bool; WIDTH]; HEIGHT],
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
    grid: [[bool; WIDTH]; HEIGHT],
    generation: usize,
}

impl Default for GameOfLifeResource {
    fn default() -> Self {
        Self {
            grid: [[false; WIDTH]; HEIGHT],
            generation: 0,
        }
    }
}

#[derive(Resource)]
struct RngResource(Rng);

// Use the concrete display type in our resource.
#[derive(Resource)]
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

fn render_system(mut display_res: ResMut<DisplayResource>, game: Res<GameOfLifeResource>) {
    display_res.display.clear(Rgb565::BLACK).unwrap();
    draw_grid(&mut display_res.display, &game.grid).unwrap();
    write_generation(&mut display_res.display, game.generation).unwrap();
}

// --- Main Function ---

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 72 * 1024);
    init_logger_from_env();

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
    .with_mosi(peripherals.GPIO6);
    let cs_output = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    // Create a proper SPI device with a Delay instance.
    let spi_delay = Delay::new();
    let spi_device = ExclusiveDevice::new(spi, cs_output, spi_delay).unwrap();

    // LCD interface: DC = GPIO15.
    let lcd_dc = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    // Leak a Box to obtain a 'static mutable buffer.
    let buffer: &'static mut [u8; 512] = Box::leak(Box::new([0_u8; 512]));
    let di = SpiInterface::new(spi_device, lcd_dc, buffer);

    // Create a separate Delay for display initialization.
    let mut display_delay = Delay::new();
    display_delay.delay_ns(500_000u32);

    // Reset pin: GPIO21. Per BSP, reset is active low.
    let reset = Output::new(
        peripherals.GPIO21,
        Level::Low,  // match BSP: lcd_reset_pin! creates with Level::Low.
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

    // Backlight on GPIO22: create pin with initial low then set high.
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

    let mut world = World::default();
    world.insert_resource(game);
    world.insert_resource(RngResource(rng_instance));
    world.insert_resource(DisplayResource { display });

    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    // Create a separate Delay for game-loop timing.
    let mut loop_delay = Delay::new();

    loop {
        schedule.run(&mut world);
        loop_delay.delay_ms(100u32);
    }
}
