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
    // ... (other initialization commands omitted for brevity)
    InitCmd::Cmd(0x3A, &[0x66]),  // Set color format (65K RGB)
    InitCmd::Cmd(0x11, &[]),      // Sleep out
    InitCmd::Delay(120),          // Allow time to wake up
    InitCmd::Cmd(0x29, &[]),      // Display on
    InitCmd::Delay(20),           // Stabilize display
];

/// A wrapper around a boxed array that implements FrameBufferBackend.
pub struct HeapBuffer<C: PixelColor, const N: usize>(Box<[C; N]>);

impl<C: PixelColor, const N: usize> HeapBuffer<C, N> {
    pub fn new(data: Box<[C; N]>) -> Self {
        Self(data)
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


/// Render the game state to the display using the hardware RGB interface
/// Render the game state to the display using the hardware RGB interface
fn render_system(
    mut display_res: NonSendMut<DisplayResource>,
    game: Res<GameOfLifeResource>,
    mut fb_res: ResMut<FrameBufferResource>,
) {

    // Clear the framebuffer.
    fb_res.frame_buf.clear(Rgb565::BLACK).unwrap();
    // Draw the game grid (using the age-based color) and generation number.
    draw_grid(&mut fb_res.frame_buf, &game.grid).unwrap();
    write_generation(&mut fb_res.frame_buf, game.generation).unwrap();

    // --- Overlay centered text ---
    let line1 = "Rust no_std ESP32-S3-LCD-Ev-BOARD";
    let line2 = "Bevy ECS 0.15 no_std";
    // Estimate text width: assume ~8 pixels per character.
    let line1_width = line1.len() as i32 * 8;
    let line2_width = line2.len() as i32 * 8;
    let x1 = (LCD_H_RES as i32 - line1_width) / 2 + 14;
    let x2 = (LCD_H_RES as i32 - line2_width) / 2 + 14;
    // For vertical centering, assume 26 pixels total text height.
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

    // Define the area covering the entire framebuffer.
    let area = Rectangle::new(Point::zero(), fb_res.frame_buf.size());

    // Convert framebuffer data to a slice of u16 values that can be written to the DMA buffer
    let mut pixel_data = alloc::vec![0u16; LCD_BUFFER_SIZE];

    for (i, pixel) in fb_res.frame_buf.data.iter().enumerate() {
        if i < LCD_BUFFER_SIZE {
            // Convert Rgb565 to u16
            pixel_data[i] = pixel.to_le_bytes()[0] as u16 | ((pixel.to_le_bytes()[1] as u16) << 8);
            // Alternatively, if available: pixel_data[i] = pixel.into_storage();
        }
    }

    // Write the prepared data to the DMA buffer
    // Get a mutable reference to the underlying buffer
    let buffer = display_res.dma_buf.as_mut_slice();
    for (i, &pixel) in pixel_data.iter().enumerate() {
        if i < buffer.len() {
            buffer[i] = pixel as u8;
        }
    }

    // Add any necessary flush/update call to your display hardware here
    // For example: display_res.flush().unwrap();
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

#[main]
fn main() -> ! {
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default()
        .with_cpu_clock(_240MHz));
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    init_logger_from_env();

    // --- Setup I2C for the display controller initialization ---
    let i2c = I2c::new(
        peripherals.I2C0,
        i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
        .unwrap()
        .with_sda(peripherals.GPIO47)
        .with_scl(peripherals.GPIO48);

    let mut expander = Tca9554::new(i2c);
    expander.write_output_reg(0b1111_0011).unwrap();
    expander.write_direction_reg(0b1111_0001).unwrap();

    let delay = Delay::new();

    println!("Initializing display controller");

    // Initialize the display controller via I2C expander
    let mut write_byte = |b: u8, is_cmd: bool| {
        const SCS_BIT: u8 = 0b0000_0010;
        const SCL_BIT: u8 = 0b0000_0100;
        const SDA_BIT: u8 = 0b0000_1000;

        let mut output = 0b1111_0001 & !SCS_BIT;
        expander.write_output_reg(output).unwrap();

        for bit in core::iter::once(!is_cmd).chain((0..8).map(|i| (b >> i) & 0b1 != 0).rev()) {
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

    // Send initialization commands to the display controller
    for &init in INIT_CMDS.iter() {
        match init {
            InitCmd::Cmd(cmd, args) => {
                write_byte(cmd, true);
                for &arg in args {
                    write_byte(arg, false);
                }
            }
            InitCmd::Delay(ms) => {
                delay.delay_millis(ms as u32);
            }
        }
    }
    drop(vsync_must_be_high_during_setup);

    // --- Configure the LCD_CAM peripheral for RGB interface ---
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Configure the RGB interface
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

    // Initialize the RGB parallel display interface
    let dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
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

    println!("Display initialized");

    // --- Initialize ECS Resources ---
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(8912);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // Setup button
    let button = Input::new(
        peripherals.GPIO0,  // Adjust to match your board's button pin
        InputConfig::default().with_pull(Pull::Up),
    );
    let button_resource = ButtonResource { button };

    // Create the framebuffer resource with HeapBuffer
    let fb_data = Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
    let fb_buffer = HeapBuffer::new(fb_data);
    let frame_buf = FrameBuf::new(fb_buffer, LCD_H_RES as usize, LCD_V_RES as usize);
    let fb_resource = FrameBufferResource { frame_buf };

    // Create the DMA buffer using DMA-compatible buffers
    let dma_buf = dma_tx_buf;

    // Create display buffer using HeapBuffer
    let display_data = Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
    let display_buffer = HeapBuffer::new(display_data);
    let display = FrameBuf::new(display_buffer, LCD_H_RES as usize, LCD_V_RES as usize);

    // Create the display resource
    let mut display_resource = DisplayResource {
        display,
        dpi,
        dma_buf,
    };

    // Initialize display with blue color to confirm it's working properly
    let blue_value = Rgb565::BLUE.into_storage();
    // Fill the buffer with blue color
    // let buffer = display_resource.dma_buf.buffer();
    // for i in 0..LCD_BUFFER_SIZE.min(buffer.len()) {
    //     buffer[i] = blue_value;
    // }

    // This is the recommended approach for your code structure
    // let transfer = display_resource.dpi.send(false, &mut display_resource.dma_buf);
    // // This re-assigns the values back to your resource after the transfer completes
    // (_, display_resource.dpi, display_resource.dma_buf) = transfer.wait();

    let mut loop_delay = Delay::new();
    // Wait a moment so we can see the blue screen
    loop_delay.delay_ms(500u32);



    // Initialize RNG
    let mut rng_instance = Rng::new(peripherals.RNG);

    // Initialize game grid
    let mut game_grid = [[0u8; GRID_WIDTH]; GRID_HEIGHT];
    randomize_grid(&mut rng_instance, &mut game_grid);

    // Add a glider pattern
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        game_grid[*y][*x] = 1;
    }

    let game_resource = GameOfLifeResource {
        grid: game_grid,
        generation: 0,
    };



    // --- Initialize ECS World and Schedule ---
    let mut world = World::new();

    // Add resources to the world
    world.insert_resource(game_resource);
    world.insert_resource(RngResource(rng_instance));
    world.insert_resource(ButtonState::default());
    world.insert_resource(fb_resource);

    // Insert non-send resources
    world.insert_non_send_resource(display_resource);
    world.insert_non_send_resource(button_resource);


    // Add systems to schedule
    let mut schedule = Schedule::default();

    // Add systems to schedule
    schedule.add_systems((
        update_game_of_life_system,
        button_reset_system,
        render_system
    ).chain());


    // Main ECS loop
    loop {
        schedule.run(&mut world);

        loop_delay.delay_ms(50u32);
    }
}