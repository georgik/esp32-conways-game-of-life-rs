#![no_std]
#![no_main]

extern crate alloc;
use core::mem;
use core::slice;
use esp_hal::clock::CpuClock::_240MHz;
use alloc::boxed::Box;
use alloc::string::ToString;
use core::fmt::Write;
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_graphics_framebuf::FrameBuf;
use embedded_graphics_framebuf::backends::FrameBufferBackend;
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::delay::Delay;
use esp_hal::dma::{DmaDescriptor, DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::i2c::{self, master::I2c};
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode,
        Phase,
        Polarity,
        dpi::{Config, Dpi, Format, FrameTiming},
    },
};
use esp_hal::{
    Blocking,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    dma_loop_buffer,
    main,
    peripherals::Peripherals,
    rng::Rng,
    time::Rate,
};
use esp_println::{logger::init_logger_from_env, println};
use log::info;
use bevy_ecs::prelude::*;
use mipidsi::interface::ParallelInterface;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("Panic: {}", _info);
    loop {}
}

// --- DISPLAY CONFIGURATION ---
const LCD_H_RES: u16 = 480;
const LCD_V_RES: u16 = 480;
const BUFFER_SIZE: usize = LCD_H_RES as usize * LCD_V_RES as usize;

// --- Game of Life Definitions ---
const GRID_WIDTH: usize = 60;
const GRID_HEIGHT: usize = 60;
const RESET_AFTER_GENERATIONS: usize = 500;

// Define the I2C expander struct (based on working code)
struct Tca9554 {
    i2c: I2c<'static, Blocking>,
    address: u8,
}

impl Tca9554 {
    pub fn new(i2c: I2c<'static, Blocking>) -> Self {
        Self { i2c, address: 0x20 }
    }

    pub fn write_direction_reg(&mut self, value: u8) -> Result<(), i2c::master::Error> {
        self.i2c.write(self.address, &[0x03, value])
    }

    pub fn write_output_reg(&mut self, value: u8) -> Result<(), i2c::master::Error> {
        self.i2c.write(self.address, &[0x01, value])
    }
}

// Define initialization commands
#[derive(Copy, Clone)]
enum InitCmd {
    Cmd(u8, &'static [u8]),
    Delay(u8),
}

// Initialization commands for the display controller
const INIT_CMDS: &[InitCmd] = &[
    InitCmd::Cmd(0xf0, &[0x55, 0xaa, 0x52, 0x08, 0x00]),
    InitCmd::Cmd(0xf6, &[0x5a, 0x87]),
    InitCmd::Cmd(0xc1, &[0x3f]),
    InitCmd::Cmd(0xc2, &[0x0e]),
    InitCmd::Cmd(0xc6, &[0xf8]),
    InitCmd::Cmd(0xc9, &[0x10]),
    InitCmd::Cmd(0xcd, &[0x25]),
    InitCmd::Cmd(0xf8, &[0x8a]),
    InitCmd::Cmd(0xac, &[0x45]),
    InitCmd::Cmd(0xa0, &[0xdd]),
    InitCmd::Cmd(0xa7, &[0x47]),
    InitCmd::Cmd(0xfa, &[0x00, 0x00, 0x00, 0x04]),
    InitCmd::Cmd(0x86, &[0x99, 0xa3, 0xa3, 0x51]),
    InitCmd::Cmd(0xa3, &[0xee]),
    InitCmd::Cmd(0xfd, &[0x3c, 0x3]),
    InitCmd::Cmd(0x71, &[0x48]),
    InitCmd::Cmd(0x72, &[0x48]),
    InitCmd::Cmd(0x73, &[0x00, 0x44]),
    InitCmd::Cmd(0x97, &[0xee]),
    InitCmd::Cmd(0x83, &[0x93]),
    InitCmd::Cmd(0x9a, &[0x72]),
    InitCmd::Cmd(0x9b, &[0x5a]),
    InitCmd::Cmd(0x82, &[0x2c, 0x2c]),
    InitCmd::Cmd(0xB1, &[0x10]),
    InitCmd::Cmd(
        0x6d,
        &[
            0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a, 0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e,
            0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01, 0x09, 0x0b, 0x0d, 0x0f,
            0x1a, 0x19, 0x1f, 0x00,
        ],
    ),
    InitCmd::Cmd(
        0x64,
        &[
            0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04, 0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x65,
        &[
            0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02, 0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x66,
        &[
            0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00, 0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x67,
        &[
            0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02, 0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x68,
        &[
            0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(0x60, &[0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a]),
    InitCmd::Cmd(0x63, &[0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a]),
    InitCmd::Cmd(0x69, &[0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08]),
    InitCmd::Cmd(0x6b, &[0x07]),
    InitCmd::Cmd(0x7a, &[0x08, 0x13]),
    InitCmd::Cmd(0x7b, &[0x08, 0x13]),
    InitCmd::Cmd(
        0xd1,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd2,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd3,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd4,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd5,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(
        0xd6,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(0x3A, &[0x66]),
    InitCmd::Cmd(0x11, &[]),
    InitCmd::Delay(120),
    InitCmd::Cmd(0x29, &[]),
    InitCmd::Delay(20),

];

/// A wrapper around a boxed array that implements FrameBufferBackend.
pub struct HeapBuffer<C: PixelColor, const N: usize>(Box<[C; N]>);

impl<C: PixelColor, const N: usize> HeapBuffer<C, N> {
    pub fn new(data: Box<[C; N]>) -> Self {
        Self(data)
    }

    pub fn as_slice(&self) -> &[C] {
        self.0.as_ref()
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

// --- Resources for the ECS ---
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
struct RngResource(Rng);

// 2 DMA descriptors
const NUM_DMA_DESCRIPTORS: usize = 2;


type MyDisplay = FrameBuf<Rgb565, &'static mut [u16]>;
type HeapDisplay<const N: usize> = FrameBuf<Rgb565, HeapBuffer<Rgb565, N>>;

// Resources for display access
struct DisplayResource {
    display: FrameBuf<Rgb565, HeapBuffer<Rgb565, BUFFER_SIZE>>,
    dpi: Dpi<'static, Blocking>,
    dma_buf: DmaTxBuf,
}


// Button resources
struct ButtonResource {
    button: Input<'static>,
}

#[derive(Default, Resource)]
struct ButtonState {
    was_pressed: bool,
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

fn update_game_of_life(grid: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT]) {
    let mut new_grid = [[0u8; GRID_WIDTH]; GRID_HEIGHT];
    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            // Count neighbors: consider a cell alive if its age is >0.
            let mut alive_neighbors = 0;
            for i in 0..3 {
                for j in 0..3 {
                    if i == 1 && j == 1 { continue; }
                    let nx = (x + i + GRID_WIDTH - 1) % GRID_WIDTH;
                    let ny = (y + j + GRID_HEIGHT - 1) % GRID_HEIGHT;
                    if grid[ny][nx] > 0 {
                        alive_neighbors += 1;
                    }
                }
            }
            if grid[y][x] > 0 {
                // Live cell survives if 2 or 3 neighbors; increment age.
                if alive_neighbors == 2 || alive_neighbors == 3 {
                    new_grid[y][x] = grid[y][x].saturating_add(1);
                } else {
                    new_grid[y][x] = 0;
                }
            } else {
                // Dead cell becomes alive if exactly 3 neighbors.
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

/// Maps cell age (1..=max_age) to a color. Newborn cells are dark blue and older cells become brighter (toward white).
fn age_to_color(age: u8) -> Rgb565 {
    if age == 0 {
        Rgb565::BLACK
    } else {
        let max_age = 10;
        let a = age.min(max_age) as u32; // clamp age and use u32 for intermediate math
        let r = ((31 * a) + 5) / max_age as u32;
        let g = ((63 * a) + 5) / max_age as u32;
        let b = 31; // Keep blue channel constant
        // Convert back to u8 and return the color.
        Rgb565::new(r as u8, g as u8, b)
    }
}

fn rgb565_to_u16(color: Rgb565) -> u16 {
    // Just convert the color directly to its storage format
    color.into_storage()
}

// --- ECS Systems ---
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

/// System to check the button and reset the simulation when pressed.
fn button_reset_system(
    mut game: ResMut<GameOfLifeResource>,
    mut rng_res: ResMut<RngResource>,
    mut btn_state: ResMut<ButtonState>,
    button_res: NonSend<ButtonResource>,
) {
    // Check if the button is pressed (active low)
    if button_res.button.is_low() {
        if !btn_state.was_pressed {
            // Button press detected: reset simulation.
            randomize_grid(&mut rng_res.0, &mut game.grid);
            game.generation = 0;
            btn_state.was_pressed = true;
        }
    } else {
        btn_state.was_pressed = false;
    }
}

/// Draws the game grid using the cell age for color.
fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[u8; GRID_WIDTH]; GRID_HEIGHT],
) -> Result<(), D::Error> {
    let border_color = Rgb565::new(230, 230, 230);
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
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                    .draw(display)?;
            }
        }
    }
    Ok(())
}

const LCD_BUFFER_SIZE: usize = 480*480;

type FbBuffer = HeapBuffer<Rgb565, LCD_BUFFER_SIZE>;

type MyFrameBuf = FrameBuf<Rgb565, FbBuffer>;

#[derive(Resource)]
struct FrameBufferResource {
    frame_buf: MyFrameBuf,
}

// fn render_system(
//     mut display_res: NonSendMut<DisplayResource>,
//     game: Res<GameOfLifeResource>,
//     mut fb_res: ResMut<FrameBufferResource>,
// ) {
//     // Clear the framebuffer
//     fb_res.frame_buf.clear(Rgb565::BLACK).unwrap();
//
//     // Draw the game grid and generation number
//     draw_grid(&mut fb_res.frame_buf, &game.grid).unwrap();
//     write_generation(&mut fb_res.frame_buf, game.generation).unwrap();
//
//     // Overlay centered text
//     let line1 = "Rust no_std ESP32-S3";
//     let line2 = "Bevy ECS 0.15 no_std";
//     let line1_width = line1.len() as i32 * 8;
//     let line2_width = line2.len() as i32 * 8;
//     let x1 = (LCD_H_RES as i32 - line1_width) / 2 + 14;
//     let x2 = (LCD_H_RES as i32 - line2_width) / 2 + 14;
//     let y = (LCD_V_RES as i32 - 26) / 2;
//
//     Text::new(
//         line1,
//         Point::new(x1, y),
//         MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
//     )
//         .draw(&mut fb_res.frame_buf)
//         .unwrap();
//
//     Text::new(
//         line2,
//         Point::new(x2, y + 14),
//         MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
//     )
//         .draw(&mut fb_res.frame_buf)
//         .unwrap();
//
//     // Define the area covering the entire framebuffer
//     let area = Rectangle::new(Point::zero(), fb_res.frame_buf.size());
//
//     // Access the underlying data through our custom HeapBuffer method
//     // This is a bit hacky, but necessary since we don't have direct access to the data
//     let fb_data = unsafe {
//         // Get a reference to the HeapBuffer from the FrameBuf
//         // This assumes the HeapBuffer is stored directly in the FrameBuf
//         let frame_buf_ptr = &fb_res.frame_buf as *const _ as *const HeapBuffer<Rgb565, LCD_BUFFER_SIZE>;
//         &*frame_buf_ptr
//     };
//
//     // Use our added method to get a slice of pixels
//     display_res
//         .display
//         .fill_contiguous(&area, fb_data.as_slice().iter().copied())
//         .unwrap();
//
// }
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

#[main]
fn main() -> ! {
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default()
        .with_cpu_clock(_240MHz));
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    init_logger_from_env();

    // Setup I2C for the TCA9554 IO expander
    let i2c = I2c::new(
        peripherals.I2C0,
        i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
        .unwrap()
        .with_sda(peripherals.GPIO47)
        .with_scl(peripherals.GPIO48);

    // Initialize the IO expander for controlling the display
    let mut expander = Tca9554::new(i2c);
    expander.write_output_reg(0b1111_0011).unwrap();
    expander.write_direction_reg(0b1111_0001).unwrap();

    let delay = Delay::new();
    println!("Initializing display...");

    // Set up the write_byte function for sending commands to the display
    let mut write_byte = |b: u8, is_cmd: bool| {
        const SCS_BIT: u8 = 0b0000_0010;
        const SCL_BIT: u8 = 0b0000_0100;
        const SDA_BIT: u8 = 0b0000_1000;

        let mut output = 0b1111_0001 & !SCS_BIT;
        expander.write_output_reg(output).unwrap();

        for bit in core::iter::once(!is_cmd)
            .chain((0..8).map(|i| (b >> i) & 0b1 != 0).rev())
        {
            let prev = output;
            if bit {
                output |= SDA_BIT;
            } else {
                output &= !SDA_BIT;
            }
            if prev != output {
                expander.write_output_reg(output).unwrap();
            }

            output &= !SCL_BIT;
            expander.write_output_reg(output).unwrap();

            output |= SCL_BIT;
            expander.write_output_reg(output).unwrap();
        }

        output &= !SCL_BIT;
        expander.write_output_reg(output).unwrap();

        output &= !SDA_BIT;
        expander.write_output_reg(output).unwrap();

        output |= SCS_BIT;
        expander.write_output_reg(output).unwrap();
    };

    // VSYNC must be high during initialization
    let mut vsync_pin = peripherals.GPIO3;
    let vsync_must_be_high_during_setup =
        Output::new(&mut vsync_pin, Level::High, OutputConfig::default());

    // Initialize the display by sending the initialization commands
    for &init in INIT_CMDS.iter() {
        match init {
            InitCmd::Cmd(cmd, args) => {
                write_byte(cmd, true);
                for &arg in args {
                    write_byte(arg, false);
                }
            }
            InitCmd::Delay(ms) => {
                delay.delay_millis(ms as _);
            }
        }
    }
    drop(vsync_must_be_high_during_setup);

    // Set up DMA channel for LCD
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Create a DMA buffer for sending data to the display
    // 4 scan lines
    let mut dma_buf = dma_loop_buffer!(2 * 480);

    // Configure the RGB display
    let config = Config::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(16))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: 480,
            horizontal_total_width: 520,
            horizontal_blank_front_porch: 10,

            vertical_active_height: 480,
            vertical_total_height: 510,
            vertical_blank_front_porch: 10,

            hsync_width: 10,
            vsync_width: 10,

            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    // Initialize the DPI interface with all the pins
    let mut dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
        .unwrap()
        .with_vsync(vsync_pin)
        .with_hsync(peripherals.GPIO46)
        .with_de(peripherals.GPIO17)
        .with_pclk(peripherals.GPIO9)
        // Blue
        .with_data0(peripherals.GPIO10)
        .with_data1(peripherals.GPIO11)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13)
        .with_data4(peripherals.GPIO14)
        // Green
        .with_data5(peripherals.GPIO21)
        .with_data6(peripherals.GPIO8)
        .with_data7(peripherals.GPIO18)
        .with_data8(peripherals.GPIO45)
        .with_data9(peripherals.GPIO38)
        .with_data10(peripherals.GPIO39)
        // Red
        .with_data11(peripherals.GPIO40)
        .with_data12(peripherals.GPIO41)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO2)
        .with_data15(peripherals.GPIO1);

    println!("Display initialized, starting game logic");

    // Initialize Game of Life
    let mut game_grid = [[0u8; GRID_WIDTH]; GRID_HEIGHT];
    let mut generation = 0;
    let mut rng = Rng::new(peripherals.RNG);

    // Initialize with random state
    randomize_grid(&mut rng, &mut game_grid);

    // Create a framebuffer in RAM
    let fb_data: Box<[Rgb565; LCD_BUFFER_SIZE]> = Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
    let heap_buffer = HeapBuffer::new(fb_data);
    let mut frame_buf = FrameBuf::new(heap_buffer, LCD_H_RES.into(), LCD_V_RES.into());

    // Draw initial screen (blue for testing)
    frame_buf.clear(Rgb565::BLUE).unwrap();

    // Helper function to convert Rgb565 to raw u16 format for the display
    fn rgb565_to_u16(color: Rgb565) -> u16 {
        color.into_storage()
    }

    // Initial transfer
    let mut transfer = dpi.send(false, dma_buf).map_err(|e| e.0).unwrap();
    let mut loop_delay = Delay::new();

    println!("Starting main loop");
    let mut current_line = 0;
    let mut current_pixel = 0;


    // Main loop
    loop {
        // Wait for previous transfer to complete and get resources back
        (_, dpi, dma_buf) = transfer.wait();

        if current_line == 0 && current_pixel == 0 {

            // Update game state
            update_game_of_life(&mut game_grid);
            generation += 1;
            if generation >= RESET_AFTER_GENERATIONS {
                randomize_grid(&mut rng, &mut game_grid);
                generation = 0;
            }

            // Clear the framebuffer
            frame_buf.clear(Rgb565::BLACK).unwrap();

            // Draw the game grid and generation count
            draw_grid(&mut frame_buf, &game_grid).unwrap();
            write_generation(&mut frame_buf, generation).unwrap();

            // Overlay centered text
            let line1 = "Rust no_std ESP32-S3";
            let line2 = "Conway's Game of Life";
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
                .draw(&mut frame_buf)
                .unwrap();

            Text::new(
                line2,
                Point::new(x2, y + 14),
                MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
            )
                .draw(&mut frame_buf)
                .unwrap();
        }

        // Calculate how many pixels we can send at once
        let pixels_per_transfer = dma_buf.len() / 2; // Each pixel is 2 bytes

        // Fill the DMA buffer with pixels
        for i in 0..pixels_per_transfer {
            if current_pixel < LCD_H_RES {
                // Get the correct color from the framebuffer
                let color = frame_buf.get_color_at(Point::new(current_pixel as i32, current_line as i32));

                // Convert to u16 and write to DMA buffer
                let color_u16 = rgb565_to_u16(color);

                // Try different byte orders
                // dma_buf[i*2..i*2+2].copy_from_slice(&color_u16.to_be_bytes());
                dma_buf[i*2..i*2+2].copy_from_slice(&color_u16.to_le_bytes());

                current_pixel += 1;
            } else {
                // Fill remaining buffer with black if we've reached the end of the line
                dma_buf[i*2..i*2+2].copy_from_slice(&0u16.to_le_bytes());
            }
        }

        // Start new transfer to the display
        transfer = dpi.send(false, dma_buf).map_err(|e| e.0).unwrap();

        // Update pixel/line counters
        if current_pixel >= LCD_H_RES {
            current_pixel = 0;
            current_line += 1;

            if current_line >= LCD_V_RES as usize {
                current_line = 0;
                // Small delay between frames
                loop_delay.delay_ms(10u32);
            }
        }

    }
}