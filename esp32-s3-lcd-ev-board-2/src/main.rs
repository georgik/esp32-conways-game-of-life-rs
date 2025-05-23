#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use bevy_ecs::prelude::*;
use embedded_graphics::{
    Drawable,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_graphics_framebuf::FrameBuf;
use embedded_graphics_framebuf::backends::FrameBufferBackend;
use esp_hal::clock::CpuClock::_240MHz;
use esp_hal::delay::Delay;
use esp_hal::dma::{CHUNK_SIZE, DmaDescriptor, DmaTxBuf};
use esp_hal::i2c::{self, master::I2c};
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode, Phase, Polarity,
        dpi::{Config, Dpi, Format, FrameTiming},
    },
};
use esp_hal::{
    Blocking,
    gpio::{Level, Output, OutputConfig},
    main,
    peripherals::Peripherals,
    rng::Rng,
    time::Rate,
};
use esp_println::logger::init_logger_from_env;
use esp_println::println;
use log::{error, info};


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    error!("Panic: {}", _info);
    loop {}
}

// --- DISPLAY CONFIGURATION ---

const LCD_H_RES: u16 = 800;

const LCD_V_RES: u16 = 1;
const BUFFER_SIZE: usize = LCD_H_RES as usize * LCD_V_RES as usize;

// --- Game of Life Definitions ---
const GRID_WIDTH: usize = 140;
const GRID_HEIGHT: usize = 70;

// Define initialization commands
#[derive(Copy, Clone)]
enum InitCmd {
    Cmd(u8, &'static [u8]),
    Delay(u8),
}

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

/// Draws the game grid using the cell age for color.
fn draw_grid<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    grid: &[[u8; GRID_WIDTH]; GRID_HEIGHT],
) -> Result<(), D::Error> where <D as embedded_graphics::draw_target::DrawTarget>::Error: core::fmt::Debug {
    let size = display.bounding_box().size;
    if size.width == 0 || size.height == 0 {
        error!("Display bounding box is invalid: {:?}", size);
        return Ok(()); // skip drawing to avoid freeze
    }

    let border_color = Rgb565::new(230, 230, 230);
    for (y, row) in grid.iter().enumerate() {


        for (x, &age) in row.iter().enumerate() {
            let point = Point::new(x as i32 * 7, y as i32 * 7);
            if age > 0 {

                match Rectangle::new(point, Size::new(7, 7))
                    .into_styled(PrimitiveStyle::with_fill(border_color))
                    .draw(display)
                {
                    Ok(()) => (),
                    Err(e) => error!("Draw error at ({}, {}): {:?}", x, y, e),
                }

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

const LCD_BUFFER_SIZE: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize);

// Size of the entire frame in bytes (2 bytes per pixel)
const FRAME_BYTES: usize = BUFFER_SIZE * 2;
// Number of descriptors needed, each up to CHUNK_SIZE (4095)
const NUM_DMA_DESC: usize = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

/// Place the descriptor(s) in DMA-capable RAM.
#[unsafe(link_section = ".dma")]
static mut TX_DESCRIPTORS: [DmaDescriptor; NUM_DMA_DESC] = [DmaDescriptor::EMPTY; NUM_DMA_DESC];

#[main]
fn main() -> ! {
    let peripherals: Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(_240MHz));
    init_logger_from_env();

    info!("Initializing PSRAM");
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);


    info!("Initializing LCD");

    // VSYNC must be high during initialization
    let mut vsync_pin = peripherals.GPIO3;
    // let vsync_must_be_high_during_setup =
    //     Output::new(vsync_pin.reborrow(), Level::High, OutputConfig::default());


    // Set up DMA channel for LCD
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Configure the RGB display
    let config = Config::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(18))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: 800,
            horizontal_total_width: 800 + 40 + 48, // active + back porch + front porch
            horizontal_blank_front_porch: 48 + 40, // front + back
            vertical_active_height: 480,
            vertical_total_height: 480 + 32 + 13, // active + back + front
            vertical_blank_front_porch: 13 + 32, // front + back
            hsync_width: 40,
            vsync_width: 23,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    // Initialize the DPI interface with all the pins
    let mut dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
        .unwrap()
        .with_data0(peripherals.GPIO10)
        .with_data1(peripherals.GPIO11)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13)
        .with_data4(peripherals.GPIO14)
        .with_data5(peripherals.GPIO21)
        .with_data6(peripherals.GPIO47)
        .with_data7(peripherals.GPIO48)
        .with_data8(peripherals.GPIO45)
        .with_data9(peripherals.GPIO38)
        .with_data10(peripherals.GPIO39)
        .with_data11(peripherals.GPIO40)
        .with_data12(peripherals.GPIO41)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO2)
        .with_data15(peripherals.GPIO1)
        .with_hsync(peripherals.GPIO46)
        .with_vsync(vsync_pin.reborrow())
        .with_de(peripherals.GPIO17)
        .with_pclk(peripherals.GPIO9);




    info!("Display initialized, starting game logic");

    // Initialize Game of Life
    let mut game_grid = [[0u8; GRID_WIDTH]; GRID_HEIGHT];
    let mut rng = Rng::new(peripherals.RNG);

    // Initialize with random state
    randomize_grid(&mut rng, &mut game_grid);

    // Create a framebuffer in RAM, initialized to red to test rendering
    let fb_data: Box<[Rgb565; BUFFER_SIZE]> = Box::new([Rgb565::new(31, 0, 0); BUFFER_SIZE]);
    let heap_buffer = HeapBuffer::new(fb_data);
    let mut frame_buf = FrameBuf::new(heap_buffer, LCD_H_RES.into(), LCD_V_RES.into());

    info!("Frame buffer created");

    // Check whether buffers were allocated correctly
    if frame_buf.data.is_empty() {
        error!("Frame buffer allocation failed");
    } else {
        info!("Frame buffer allocated successfully");
    }
    // Draw initial screen (bright green for testing)
    frame_buf.clear(Rgb565::new(0, 63, 0)).unwrap(); // bright green

    // Initial transfer

    info!("Starting main loop");

    const FRAME_BYTES: usize = BUFFER_SIZE * 2;
    let buf_box: Box<[u8; FRAME_BYTES]> = Box::new([0; FRAME_BYTES]);
    let psram_buf: &'static mut [u8] = Box::leak(buf_box);

    // Check allocation
    if psram_buf.is_empty() {
        error!("PSRAM buffer allocation failed");
    } else {
        info!("PSRAM buffer allocated successfully");
    }

    // Tie to descriptor set for one-shot DMA
    let mut dma_tx: DmaTxBuf =
        unsafe { DmaTxBuf::new(&mut TX_DESCRIPTORS[..], psram_buf).unwrap() };
    info!("DMA buffer created");

    // Main loop to draw the entire image
    loop {
        info!("Frame");
        update_game_of_life(&mut game_grid);

        // Clear framebuffer to bright green before drawing (testing DMA)
        frame_buf.clear(Rgb565::new(0, 63, 0)).unwrap(); // bright green
        draw_grid(&mut frame_buf, &game_grid).unwrap();

        let dst = dma_tx.as_mut_slice();
        for (i, px) in frame_buf.data.iter().enumerate() {
            let [lo, hi] = px.into_storage().to_le_bytes();
            dst[2 * i] = lo;
            dst[2 * i + 1] = hi;
        }

        // One-shot transfer
        match dpi.send(false, dma_tx) {
            Ok(xfer) => {
                let (res, dpi2, buf2) = xfer.wait();
                dpi = dpi2;
                dma_tx = buf2;
                if let Err(e) = res {
                    error!("DMA error: {:?}", e);
                }
            }
            Err((e, dpi2, buf2)) => {
                error!("DMA send error: {:?}", e);
                dpi = dpi2;
                dma_tx = buf2;
            }
        }
    }
}
