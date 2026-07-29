//! Drivesrs for the EK79007 MIPI-DSI panel on the ESP32-P4-Function-EV-Board v1.5.
//!
//! Board wiring (fixed, no external GPIOs needed):
//! - LCD RST     => GPIO27
//! - LCD BL PWM  => GPIO26  (driven high = backlight on)
//! - MIPI-DSI    => internal (no GPIO mux)
//! - VDD_MIPI_DPHY powered by the driver via PMU LDO3

//% CHIP_FILTER: mipi_dsi_driver_supported

#![no_std]
#![no_main]

const H_ACTIVE: u32 = 1024;
const V_ACTIVE: u32 = 600;

const BYTES_PER_PIXEL: usize = 2;
const FB_SIZE: usize = H_ACTIVE as usize * V_ACTIVE as usize * BYTES_PER_PIXEL;

//GAME constants
const GRID_WIDTH: usize = 85;
const GRID_HEIGHT: usize = 50;

const CELL_SIZE: usize = 12;
const CELL_INSET: usize = 1;

const RESET_AFTER_GENERATIONS: usize = 300;
const FRAME_DELAY_MS: u32 = 80;

type GameGrid = [[u8; GRID_WIDTH]; GRID_HEIGHT];

extern crate alloc;

use core::alloc::Layout;

use esp_alloc as _;
use esp_backtrace as _;

use esp_println::println;

use esp_hal::{
    clock::{
        CpuClock,
        ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    },
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    mipi_dsi::{
        Config, DataLanes, MipiDsi,
        dpi::{ColorFormat, DpiClockSource, DpiConfig, FrameTiming},
    },
    peripherals::Peripherals,
    psram,
};

use core::{convert::Infallible, fmt::Write};

use embedded_graphics::{
    Drawable,
    geometry::{OriginDimensions, Size},
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::{Rgb565, RgbColor},
    prelude::{DrawTarget, IntoStorage, Pixel, Point},
    text::{Baseline, Text},
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals: Peripherals = esp_hal::init(config);

    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        psram::PsramConfig::default()
    );

    println!("PSRAM ready");

    let delay = Delay::new();

    // ── LCD reset (GPIO27, active-low) ──────────────────────────────────────
    let mut lcd_rst = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());
    delay.delay_millis(10);
    lcd_rst.set_high();
    delay.delay_millis(120);

    // ── Backlight on (GPIO26, active-high) ─────────────────────────────────
    let _lcd_bl = Output::new(peripherals.GPIO26, Level::High, OutputConfig::default());

    // ── MIPI DSI bus ────────────────────────────────────────────────────────
    let mut bus = MipiDsi::new(
        peripherals.MIPI_DSI,
        peripherals.VDMA_CH0,
        Config::default()
            .with_num_data_lanes(DataLanes::_2)
            .with_lane_bit_rate_mbps(1000.0)
            .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                MipiDsiPhyPllRefclkSclk::Xtal,
                0,
            ))
            .with_force_clock_lane_hs(false),
    )
    .expect("MipiDsi init failed");

    println!("DSI bus up");

    // ── EK79007 init via DBI (LP command mode) ──────────────────────────────
    {
        let mut dbi = bus.dbi(0);
        for &(cmd, params) in EK79007_INIT {
            dbi.write_cmd(cmd, params).unwrap();
        }
        // Sleep out: 120 ms settle time.
        dbi.write_cmd(0x11, &[]).unwrap();
        delay.delay_millis(120);
        // Display on.
        dbi.write_cmd(0x29, &[]).unwrap();
        delay.delay_millis(20);
    }

    println!("Panel init done");

    // ── Frame buffer in PSRAM (64-byte aligned) ─────────────────────────────
    let layout = Layout::from_size_align(FB_SIZE, 64).unwrap();
    let fb_ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
    assert!(!fb_ptr.is_null(), "PSRAM alloc failed");
    let fb1: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(fb_ptr, FB_SIZE) };

    let fb_ptr = unsafe { alloc::alloc::alloc_zeroed(layout) };
    assert!(!fb_ptr.is_null(), "PSRAM alloc failed");
    let fb2: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(fb_ptr, FB_SIZE) };
    let fbs: [&mut [u8]; 2] = [fb1, fb2];

    // ── Enter video mode (DPI) ──────────────────────────────────────────────
    let dpi_cfg = DpiConfig {
        virtual_channel: 0,
        pixel_clock_mhz: 48.0,
        dpi_clk_src: DpiClockSource::PllF240m,
        in_color_format: ColorFormat::Rgb565,
        out_color_format: ColorFormat::Rgb565,
        timing: FrameTiming {
            h_active: H_ACTIVE,
            hsw: 10,
            hbp: 120,
            hfp: 120,
            v_active: V_ACTIVE,
            vsw: 1,
            vbp: 20,
            vfp: 20,
        },
    };

    let mut dpi = bus.dpi(dpi_cfg, &fbs).expect("DPI init failed");

    println!("Streaming");

    // ── MAIN GAME LOOP ───────────────────────────────────────────────────

    let mut rng = XorShift32::new(0xA5C3_91E7);

    let mut grid: GameGrid = [[0; GRID_WIDTH]; GRID_HEIGHT];
    let mut next_grid: GameGrid = [[0; GRID_WIDTH]; GRID_HEIGHT];

    randomize_grid(&mut rng, &mut grid);

    // Add a glider near the center.
    let glider_x = GRID_WIDTH / 2;
    let glider_y = GRID_HEIGHT / 2;

    let glider = [(1usize, 0usize), (2, 1), (0, 2), (1, 2), (2, 2)];

    for &(x, y) in &glider {
        grid[glider_y + y][glider_x + x] = 1;
    }

    let mut generation: usize = 0;

    println!("Starting Conway's Game of Life");

    loop {
        // Wait until vertical blanking before modifying the back buffer.
        dpi.wait_for_vsync();

        // Draw into the currently unused framebuffer.
        let back = dpi.framebuffer_mut();

        // draw_game() clears and redraws the whole framebuffer.
        draw_game(back, &grid);

        // Draw the text afterward so it appears above the game.
        draw_generation_text(back, generation);

        // Flush the PSRAM cache and switch the DMA to this framebuffer.
        dpi.commit();

        if generation % 25 == 0 {
            println!("Generation: {generation}");
        }

        // Limit the simulation to approximately ten generations per second.
        delay.delay_millis(FRAME_DELAY_MS);

        update_game_of_life(&mut grid, &mut next_grid);
        generation += 1;

        if generation >= RESET_AFTER_GENERATIONS {
            println!("Randomizing grid");

            randomize_grid(&mut rng, &mut grid);
            next_grid = [[0; GRID_WIDTH]; GRID_HEIGHT];
            generation = 0;
        }
    }
}

// ── EK79007 vendor init sequence ─────────────────────────────────────────────

static EK79007_INIT: &[(u8, &[u8])] = &[
    (0xB2, &[0x10]), // Pad control - two lanes
    (0x80, &[0x8B]),
    (0x81, &[0x78]),
    (0x82, &[0x84]),
    (0x83, &[0x88]),
    (0x84, &[0xA8]),
    (0x85, &[0xE3]),
    (0x86, &[0x88]),
];

struct XorShift32 {
    state: u32,
}

impl XorShift32 {
    fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 0x1234_5678 } else { seed },
        }
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.state;

        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;

        self.state = x;
        x
    }
}

fn randomize_grid(rng: &mut XorShift32, grid: &mut GameGrid) {
    for row in grid.iter_mut() {
        for cell in row.iter_mut() {
            // Approximately 25% of the cells start alive.
            *cell = if rng.next_u32() & 3 == 0 { 1 } else { 0 };
        }
    }
}

fn update_game_of_life(grid: &mut GameGrid, next: &mut GameGrid) {
    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let mut alive_neighbors = 0u8;

            for dy in 0..3 {
                for dx in 0..3 {
                    if dx == 1 && dy == 1 {
                        continue;
                    }

                    // Wrap around at the edges.
                    let nx = (x + dx + GRID_WIDTH - 1) % GRID_WIDTH;
                    let ny = (y + dy + GRID_HEIGHT - 1) % GRID_HEIGHT;

                    if grid[ny][nx] > 0 {
                        alive_neighbors += 1;
                    }
                }
            }

            next[y][x] = if grid[y][x] > 0 {
                if alive_neighbors == 2 || alive_neighbors == 3 {
                    // Cell survives and becomes older.
                    grid[y][x].saturating_add(1)
                } else {
                    // Cell dies.
                    0
                }
            } else if alive_neighbors == 3 {
                // New cell is born.
                1
            } else {
                0
            };
        }
    }

    core::mem::swap(grid, next);
}

fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    let r5 = (r as u16 >> 3) & 0x1f;
    let g6 = (g as u16 >> 2) & 0x3f;
    let b5 = (b as u16 >> 3) & 0x1f;

    (r5 << 11) | (g6 << 5) | b5
}

/// Convert cell age into an RGB565 color.
///
/// New cells start blue. Older cells gradually become brighter and
/// eventually approach white.
fn age_to_color(age: u8) -> u16 {
    if age == 0 {
        return rgb565(0, 0, 0);
    }

    const MAX_AGE: u32 = 10;

    let age = u32::from(age).min(MAX_AGE);

    let red = (255 * age / MAX_AGE) as u8;
    let green = (255 * age / MAX_AGE) as u8;
    let blue = 255;

    rgb565(red, green, blue)
}

fn fill_rectangle_rgb565(
    framebuffer: &mut [u8],
    x: usize,
    y: usize,
    width: usize,
    height: usize,
    color: u16,
) {
    let screen_width = H_ACTIVE as usize;
    let screen_height = V_ACTIVE as usize;

    if x >= screen_width || y >= screen_height {
        return;
    }

    let x_end = x.saturating_add(width).min(screen_width);
    let y_end = y.saturating_add(height).min(screen_height);

    let color_bytes = color.to_le_bytes();
    let stride = screen_width * BYTES_PER_PIXEL;

    for screen_y in y..y_end {
        let start = screen_y * stride + x * BYTES_PER_PIXEL;
        let end = screen_y * stride + x_end * BYTES_PER_PIXEL;

        for pixel in framebuffer[start..end].chunks_exact_mut(2) {
            pixel.copy_from_slice(&color_bytes);
        }
    }
}

fn draw_game(framebuffer: &mut [u8], grid: &GameGrid) {
    const BORDER_COLOR: u16 = 0x7BEF; // Medium gray RGB565

    let grid_pixel_width = GRID_WIDTH * CELL_SIZE;
    let grid_pixel_height = GRID_HEIGHT * CELL_SIZE;

    let offset_x = (H_ACTIVE as usize - grid_pixel_width) / 2;
    let offset_y = (V_ACTIVE as usize - grid_pixel_height) / 2;

    // Clear the entire screen to black.
    framebuffer.fill(0);

    for (grid_y, row) in grid.iter().enumerate() {
        for (grid_x, &age) in row.iter().enumerate() {
            if age == 0 {
                continue;
            }

            let x = offset_x + grid_x * CELL_SIZE;
            let y = offset_y + grid_y * CELL_SIZE;

            // Draw the cell's border.
            fill_rectangle_rgb565(framebuffer, x, y, CELL_SIZE, CELL_SIZE, BORDER_COLOR);

            // Draw the colored interior.
            fill_rectangle_rgb565(
                framebuffer,
                x + CELL_INSET,
                y + CELL_INSET,
                CELL_SIZE - CELL_INSET * 2,
                CELL_SIZE - CELL_INSET * 2,
                age_to_color(age),
            );
        }
    }
}

/// Adapter allowing embedded-graphics to draw directly into the MIPI
/// RGB565 framebuffer.
struct MipiFrameBuffer<'a> {
    data: &'a mut [u8],
    width: usize,
    height: usize,
}

impl<'a> MipiFrameBuffer<'a> {
    fn new(data: &'a mut [u8], width: usize, height: usize) -> Self {
        assert!(data.len() >= width * height * BYTES_PER_PIXEL);

        Self {
            data,
            width,
            height,
        }
    }
}

impl OriginDimensions for MipiFrameBuffer<'_> {
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}

impl DrawTarget for MipiFrameBuffer<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            // Ignore pixels outside the framebuffer.
            if point.x < 0 || point.y < 0 {
                continue;
            }

            let x = point.x as usize;
            let y = point.y as usize;

            if x >= self.width || y >= self.height {
                continue;
            }

            let index = (y * self.width + x) * BYTES_PER_PIXEL;
            let bytes = color.into_storage().to_le_bytes();

            self.data[index] = bytes[0];
            self.data[index + 1] = bytes[1];
        }

        Ok(())
    }
}

fn draw_generation_text(framebuffer: &mut [u8], generation: usize) {
    const RIGHT_MARGIN: usize = 8;
    const TOP_MARGIN: usize = 4;
    const FONT_WIDTH: usize = 8;
    const STATUS_HEIGHT: usize = 21;

    // Form4t without using the heap.
    let mut text = heapless::String::<32>::new();
    write!(&mut text, "Generation: {generation}").unwrap();

    let text_width = text.len() * FONT_WIDTH;

    // Right-align the text.
    let text_x = (H_ACTIVE as usize)
        .saturating_sub(RIGHT_MARGIN)
        .saturating_sub(text_width);

    // Draw a small black rectangle behind the text so cells don't make it
    // difficult to read.
    let background_x = text_x.saturating_sub(4);
    let background_width = text_width + 8;

    fill_rectangle_rgb565(
        framebuffer,
        background_x,
        0,
        background_width,
        STATUS_HEIGHT,
        0x0000, // RGB565 black
    );

    let mut target = MipiFrameBuffer::new(framebuffer, H_ACTIVE as usize, V_ACTIVE as usize);

    let style = MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE);

    Text::with_baseline(
        text.as_str(),
        Point::new(text_x as i32, TOP_MARGIN as i32),
        style,
        Baseline::Top,
    )
    .draw(&mut target)
    .unwrap();
}
