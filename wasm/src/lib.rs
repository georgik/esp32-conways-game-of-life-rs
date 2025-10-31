#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::cell::RefCell;
use core::fmt::Write;

use bevy_ecs::prelude::*; // includes Resource, NonSendMut, etc.
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_graphics_framebuf::FrameBuf;
extern crate getrandom;
use log::info;
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement, ImageData, window};

// === LCD & Framebuffer settings ===
const LCD_H_RES: usize = 320;
const LCD_V_RES: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES * LCD_V_RES;

// We store pixels as Rgb565.
pub type FbBuffer = [Rgb565; LCD_BUFFER_SIZE];
// Type alias for our framebuffer.
pub type MyFrameBuf = FrameBuf<Rgb565, FbBuffer>;

// === FrameBuffer Resource ===
#[derive(Resource)]
struct FrameBufferResource {
    frame_buf: MyFrameBuf,
}

impl FrameBufferResource {
    fn new() -> Self {
        // Allocate framebuffer data on the heap.
        let fb_data: FbBuffer = *Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
        let frame_buf = MyFrameBuf::new(fb_data, LCD_H_RES, LCD_V_RES);
        Self { frame_buf }
    }
}

// === Game of Life Definitions ===
// Our simulation grid is 64x48; each cell is a u8 (0 = dead, >0 = alive/age).
const GRID_WIDTH: usize = 64;
const GRID_HEIGHT: usize = 48;
const RESET_AFTER_GENERATIONS: usize = 500;

#[derive(Resource)]
struct GameOfLifeResource {
    grid: [[u8; GRID_WIDTH]; GRID_HEIGHT],
    generation: usize,
}

impl Default for GameOfLifeResource {
    fn default() -> Self {
        // Create an empty grid.
        let mut grid = [[0u8; GRID_WIDTH]; GRID_HEIGHT];
        // Seed the RNG using your helper.
        let mut rng = ChaCha8Rng::seed_from_u64(get_seed() as u64);
        // Fill the grid with random values.
        randomize_grid(&mut rng, &mut grid);
        Self {
            grid,
            generation: 0,
        }
    }
}

// Helper function that returns a u32 seed using getrandom.
fn get_seed() -> u32 {
    let mut buf = [0u8; 4];
    getrandom::fill(&mut buf).expect("failed to get random seed");
    u32::from_le_bytes(buf)
}

use rand_chacha::ChaCha8Rng;
use rand_core::{RngCore, SeedableRng};
#[derive(Resource)]
struct RngResource(ChaCha8Rng);

impl Default for RngResource {
    fn default() -> Self {
        // Get an 8-byte seed from the browser.
        let mut seed_buf = [0u8; 8];
        getrandom::fill(&mut seed_buf).expect("failed to get random seed");
        let seed = u64::from_le_bytes(seed_buf);
        Self(ChaCha8Rng::seed_from_u64(seed))
    }
}
// === Display Resource for WASM ===
// For the WASM version, our display is simulated by updating an HTML canvas.
struct DisplayResource {
    ctx: CanvasRenderingContext2d,
}

impl DisplayResource {
    fn new(ctx: CanvasRenderingContext2d) -> Self {
        Self { ctx }
    }
}

// === Simulation Functions ===

fn randomize_grid(rng: &mut impl RngCore, grid: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT]) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            // Use next_u32() to generate a random number.
            *cell = if (rng.next_u32() & 1) != 0 { 1 } else { 0 };
        }
    }
}

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
            } else if alive_neighbors == 3 {
                new_grid[y][x] = 1;
            }
        }
    }
    *grid = new_grid;
}

/// Maps cell age to a color. Young cells are dark blue and older cells brighten.
fn age_to_color(age: u8) -> Rgb565 {
    if age == 0 {
        Rgb565::BLACK
    } else {
        let max_age = 10;
        let a = age.min(max_age) as u32;
        let r = ((31 * a) + 5) / (max_age as u32);
        let g = ((63 * a) + 5) / (max_age as u32);
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
            let point = Point::new(x as i32 * 7, y as i32 * 7);
            if age > 0 {
                Rectangle::new(point, Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)?;
                Rectangle::new(point + Point::new(1, 1), Size::new(5, 5))
                    .into_styled(PrimitiveStyle::with_fill(age_to_color(age)))
                    .draw(display)?;
            } else {
                Rectangle::new(point, Size::new(7, 7))
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
    let mut num_str = String::new();
    write!(&mut num_str, "{}", generation).unwrap();
    Text::new(
        &num_str,
        Point::new(8, 13),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    .draw(display)?;
    Ok(())
}

// === ECS Systems ===
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

fn render_system(
    display_res: NonSendMut<DisplayResource>,
    game: Res<GameOfLifeResource>,
    mut fb_res: ResMut<FrameBufferResource>,
) {
    fb_res.frame_buf.clear(Rgb565::BLACK).unwrap();
    draw_grid(&mut fb_res.frame_buf, &game.grid).unwrap();
    write_generation(&mut fb_res.frame_buf, game.generation).unwrap();

    // Overlay centered text.
    let line1 = "Rust no_std WASM";
    let line2 = "Bevy ECS 0.17";
    let line1_width = line1.len() as i32 * 8;
    let line2_width = line2.len() as i32 * 8;
    let x1 = (LCD_H_RES as i32 - line1_width) / 2;
    let x2 = (LCD_H_RES as i32 - line2_width) / 2;
    let y = (LCD_V_RES as i32 - 26) / 2;
    Text::new(
        line1,
        Point::new(x1, y),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    .draw(&mut fb_res.frame_buf)
    .unwrap();
    Text::new(
        line2,
        Point::new(x2, y + 14),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    .draw(&mut fb_res.frame_buf)
    .unwrap();

    // Flush the framebuffer to the canvas.
    update_canvas(&display_res.ctx, &fb_res.frame_buf);
}

// === WASM Helper: Update the HTML canvas from the framebuffer ===
fn update_canvas(ctx: &CanvasRenderingContext2d, fb: &MyFrameBuf) {
    let mut out = Vec::with_capacity(LCD_H_RES * LCD_V_RES * 4);
    for pixel in fb.data.iter().copied() {
        let raw = pixel.into_storage();
        let r = ((raw >> 11) & 0x1F) * 255 / 31;
        let g = ((raw >> 5) & 0x3F) * 255 / 63;
        let b = (raw & 0x1F) * 255 / 31;
        out.push(r as u8);
        out.push(g as u8);
        out.push(b as u8);
        out.push(255);
    }
    let clamped = wasm_bindgen::Clamped(&out[..]);
    let image_data =
        ImageData::new_with_u8_clamped_array_and_sh(clamped, LCD_H_RES as u32, LCD_V_RES as u32)
            .unwrap();
    ctx.put_image_data(&image_data, 0.0, 0.0).unwrap();
}

#[wasm_bindgen(start)]
pub fn start() -> Result<(), JsValue> {
    console_error_panic_hook::set_once();
    info!("Starting WASM Conway’s Game of Life");

    // Set up the canvas.
    let document = window().unwrap().document().unwrap();
    let canvas = document.create_element("canvas")?;
    canvas.set_attribute("width", &LCD_H_RES.to_string())?;
    canvas.set_attribute("height", &LCD_V_RES.to_string())?;
    document.body().unwrap().append_child(&canvas)?;
    let canvas: HtmlCanvasElement = canvas.dyn_into()?;
    let ctx = canvas
        .get_context("2d")?
        .unwrap()
        .dyn_into::<CanvasRenderingContext2d>()?;

    // Create ECS world and insert resources.
    let mut world = World::default();
    world.insert_resource(GameOfLifeResource::default());
    world.insert_resource(RngResource::default());
    world.insert_resource(FrameBufferResource::new());
    // Our DisplayResource is non-send because it contains non-Sync data.
    world.insert_non_send_resource(DisplayResource::new(ctx));

    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    // --- Frame Rate Limiter Setup ---
    let frame_interval_ms = 100.0;
    let last_frame = Rc::new(RefCell::new(0.0));

    // Create an Rc<RefCell<Option<Closure<...>>>> to store the animation loop closure.
    let f: Rc<RefCell<Option<Closure<dyn FnMut()>>>> = Rc::new(RefCell::new(None));
    // Clone the Rc so we can move the clone into the closure.
    let f_clone = f.clone();

    let win = window().unwrap();
    let win_clone = win.clone();

    // Set up our animation closure.
    *f.borrow_mut() = Some(Closure::wrap(Box::new({
        let last_frame = last_frame.clone();
        move || {
            // Get the current time in milliseconds.
            let now = win_clone.performance().unwrap().now();
            {
                let mut last = last_frame.borrow_mut();
                // Only update if enough time has passed.
                if now - *last >= frame_interval_ms {
                    schedule.run(&mut world);
                    *last = now;
                }
            }
            // Schedule the next frame using the clone.
            win_clone
                .request_animation_frame(
                    f_clone.borrow().as_ref().unwrap().as_ref().unchecked_ref(),
                )
                .unwrap();
        }
    }) as Box<dyn FnMut()>));

    // Kick off the animation loop.
    win.request_animation_frame(f.borrow().as_ref().unwrap().as_ref().unchecked_ref())?;
    Ok(())
}
