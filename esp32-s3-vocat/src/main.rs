//! Conway's Game of Life on the ESP-VoCat (ESP32-S3).
//!
//! The round 360 × 360 ST77916 LCD is driven through the `esp-vocat` board
//! support crate: the simulation runs on a Bevy ECS schedule and renders into
//! the controller's frame buffer (which lives in the 16 MB octal PSRAM), then
//! only the rows that changed since the last frame are streamed over the
//! QSPI bus with `Lcd::flush`.
//!
//! The pattern starts from a random field with a glider for the first frames;
//! the generation counter is drawn in the top-left corner.
#![no_std]
#![no_main]

extern crate alloc;

// ESP-IDF App Descriptor required by newer espflash
esp_bootloader_esp_idf::esp_app_desc!();

use bevy_ecs::prelude::*;
use core::fmt::Write;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use log::{error, info};

use esp_vocat::board;
use esp_vocat::display::{Backlight, FrameBuffer, Lcd};

// --- Game of life definitions ---
// Each cell is 7 px, which fits a 51 × 51 grid inside the 360 × 360 panel.
const CELL: usize = 7;
const GRID_WIDTH: usize = 51;
const GRID_HEIGHT: usize = 51;
const RESET_AFTER_GENERATIONS: usize = 500;

/// A full-screen RGB565 frame buffer. On the ESP-VoCat this is the
/// `esp-vocat` controller buffer (253 KiB) living in PSRAM.
type FbBuffer = FrameBuffer;

/// The frame buffer, held as a normal Bevy resource so it can be drawn into
/// from a system and flushed from the main loop.
#[derive(Resource)]
struct FrameBufferResource {
    frame_buf: FbBuffer,
}

/// Game state kept as a normal (Send + Sync) Bevy resource.
#[derive(Resource)]
struct GameOfLifeResource {
    grid: [[u8; GRID_WIDTH]; GRID_HEIGHT],
    generation: usize,
}

impl Default for GameOfLifeResource {
    fn default() -> Self {
        Self {
            grid: [[0; GRID_WIDTH]; GRID_HEIGHT],
            generation: 0,
        }
    }
}

#[derive(Resource)]
struct RngResource(esp_hal::rng::Rng);

/// Spawns a fully random grid.
fn randomize_grid(rng: &mut esp_hal::rng::Rng, grid: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            let mut buf = [0u8; 1];
            rng.read(&mut buf);
            *cell = if buf[0] & 1 != 0 { 1 } else { 0 };
        }
    }
}

/// Advances the simulation by one generation (Moore neighbourhood, wrapping).
fn update_game_of_life(grid: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT]) {
    let mut new_grid = [[0u8; GRID_WIDTH]; GRID_HEIGHT];
    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let mut alive_neighbors = 0;
            for i in 0..3 {
                for j in 0..3 {
                    if i == 1 && j == 1 {
                        continue;
                    }
                    let nx = (x + i + GRID_WIDTH - 1) % GRID_WIDTH;
                    let ny = (y + j + GRID_HEIGHT - 1) % GRID_HEIGHT;
                    if grid[ny][nx] > 0 {
                        alive_neighbors += 1;
                    }
                }
            }
            if grid[y][x] > 0 {
                if alive_neighbors == 2 || alive_neighbors == 3 {
                    new_grid[y][x] = grid[y][x].saturating_add(1);
                } else {
                    new_grid[y][x] = 0;
                }
            } else {
                if alive_neighbors == 3 {
                    new_grid[y][x] = 1;
                } else {
                    new_grid[y][x] = 0;
                }
            }
        }
    }
    *grid = new_grid;
}

/// Maps cell age to a colour: newborn cells are dark and brighten towards white.
fn age_to_color(age: u8) -> Rgb565 {
    if age == 0 {
        Rgb565::BLACK
    } else {
        let max_age = 10;
        let a = age.min(max_age) as u32;
        let r = ((31 * a) + 5) / max_age as u32;
        let g = ((63 * a) + 5) / max_age as u32;
        let b = 31;
        Rgb565::new(r as u8, g as u8, b)
    }
}

fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[u8; GRID_WIDTH]; GRID_HEIGHT],
) -> Result<(), D::Error> {
    let border_color = Rgb565::new(230, 230, 230);
    for (y, row) in grid.iter().enumerate() {
        for (x, &age) in row.iter().enumerate() {
            let point = Point::new(x as i32 * CELL as i32, y as i32 * CELL as i32);
            if age > 0 {
                Rectangle::new(point, Size::new(CELL as u32, CELL as u32))
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;
                Rectangle::new(
                    point + Point::new(1, 1),
                    Size::new(CELL as u32 - 2, CELL as u32 - 2),
                )
                .into_styled(PrimitiveStyle::with_fill(age_to_color(age)))
                .draw(display)?;
            } else {
                Rectangle::new(point, Size::new(CELL as u32, CELL as u32))
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
    write!(num_str, "{generation}").unwrap();
    Text::new(
        num_str.as_str(),
        Point::new(8, 13),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    .draw(display)?;
    Ok(())
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

/// Renders one frame into the controller buffer.
fn render_system(mut fb: ResMut<FrameBufferResource>, game: Res<GameOfLifeResource>) {
    fb.frame_buf.clear(Rgb565::BLACK).unwrap();
    draw_grid(&mut fb.frame_buf, &game.grid).unwrap();
    write_generation(&mut fb.frame_buf, game.generation).unwrap();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    esp_println::println!("Panic: {_info}");
    loop {}
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = board::init();
    esp_println::logger::init_logger_from_env();

    // Heap: all of the octal PSRAM, which holds the 253 KiB framebuffer.
    let psram = board::psram(peripherals.PSRAM);
    esp_alloc::psram_allocator!(&psram);

    // Embassy timer for `embassy_time`, started through esp-rtos.
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Power the LCD (VCC_3V3) and the codec (CODEC_3V3); keep the pins alive.
    let _power = board::peripheral_power_on(peripherals.GPIO9);
    let _codec = board::codec_power_on(peripherals.GPIO48);
    Timer::after(Duration::from_millis(100)).await;

    info!("ESP-VoCat Conway's Game of Life starting");

    // LCD: reset line, backlight, then initialise the ST77916.
    let mut reset = board::LcdReset::new(peripherals.GPIO47);
    let mut backlight = Backlight::new(peripherals.LEDC, peripherals.GPIO44);
    backlight.set(100);
    let mut lcd = match Lcd::new(
        peripherals.SPI2,
        peripherals.DMA_CH0,
        esp_vocat::lcd_pins!(peripherals),
        &mut reset,
    )
    .await
    {
        Ok(l) => l,
        Err(e) => {
            error!("LCD init failed: {e:?}");
            loop {}
        }
    };

    // Initial game state: a random field plus a glider for the first frames.
    let mut game = GameOfLifeResource::default();
    let mut rng = esp_hal::rng::Rng::new();
    randomize_grid(&mut rng, &mut game.grid);
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        game.grid[*y][*x] = 1;
    }

    let mut world = World::default();
    world.insert_resource(FrameBufferResource {
        frame_buf: FrameBuffer::new(),
    });
    world.insert_resource(game);
    world.insert_resource(RngResource(rng));

    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    loop {
        // Advance the simulation and render into the controller buffer.
        schedule.run(&mut world);
        // Stream only the changed rows over QSPI.
        let mut fb = world.get_resource_mut::<FrameBufferResource>().unwrap();
        lcd.flush(&mut fb.frame_buf).await.unwrap();
        Timer::after(Duration::from_millis(50)).await;
    }
}
