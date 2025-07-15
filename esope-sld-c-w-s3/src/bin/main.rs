#![no_std]
#![no_main]
use core::fmt::Write;
use embedded_graphics::mono_font::{ascii::FONT_8X13, MonoTextStyle};
use embedded_graphics::text::Text;
use heapless::String;

use esp_hal::dma::ExternalBurstConfig;

use alloc::boxed::Box;
use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    Drawable,
};
use embedded_graphics_framebuf::backends::FrameBufferBackend;
use embedded_graphics_framebuf::FrameBuf;
use esp_hal::gpio::Level;
use esp_hal::gpio::{Output, OutputConfig};
use esp_hal::lcd_cam::{
    lcd::{
        dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
        ClockMode, Phase, Polarity,
    },
    LcdCam,
};
use esp_hal::time::Rate;
use esp_println::logger::init_logger_from_env;

use eeprom24x::{Eeprom24x, SlaveAddr};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Ticker;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::I2c;
use esp_hal::timer::{timg::TimerGroup, AnyTimer};
use esp_hal_embassy::Executor;
use log::{error, info};
use static_cell::StaticCell;

// DMA line‐buffer for parallel RGB (1 descriptor, up to 4095 bytes each)
use esp_hal::dma::{DmaDescriptor, DmaTxBuf, CHUNK_SIZE};
use esp_println::println;

use esp_hal::rng::Rng;
use esp_hal::system::{CpuControl, Stack};

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    error!("Panic: {}", _info);
    loop {}
}

extern crate alloc;

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

/// FrameBufferBackend wrapper for a PSRAM-backed [Rgb565; N] slice.
pub struct PSRAMFrameBuffer<'a> {
    buf: &'a mut [Rgb565; LCD_BUFFER_SIZE],
}

impl<'a> PSRAMFrameBuffer<'a> {
    pub fn new(buf: &'a mut [Rgb565; LCD_BUFFER_SIZE]) -> Self {
        Self { buf }
    }
}

impl<'a> FrameBufferBackend for PSRAMFrameBuffer<'a> {
    type Color = Rgb565;
    fn set(&mut self, index: usize, color: Self::Color) {
        self.buf[index] = color;
    }
    fn get(&self, index: usize) -> Self::Color {
        self.buf[index]
    }
    fn nr_elements(&self) -> usize {
        LCD_BUFFER_SIZE
    }
}

const GRID_WIDTH: usize = 46;
const GRID_HEIGHT: usize = 34;

fn update_game_of_life(
    current: &[[u8; GRID_WIDTH]; GRID_HEIGHT],
    next: &mut [[u8; GRID_WIDTH]; GRID_HEIGHT],
) {
    for y in 0..GRID_HEIGHT {
        for x in 0..GRID_WIDTH {
            let mut neighbors = 0;
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx >= 0 && nx < GRID_WIDTH as i32 && ny >= 0 && ny < GRID_HEIGHT as i32 {
                        if current[ny as usize][nx as usize] > 0 {
                            neighbors += 1;
                        }
                    }
                }
            }
            let cell = current[y][x];
            next[y][x] = match (cell > 0, neighbors) {
                (true, 2) | (true, 3) => cell.saturating_add(1), // stay alive, age
                (false, 3) => 1,                                 // born
                _ => 0,                                          // dead
            };
        }
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
fn write_generation<D: DrawTarget<Color = Rgb565>>(
    display: &mut D,
    generation: usize,
) -> Result<(), D::Error> {
    let mut num_str = String::<20>::new();
    write!(num_str, "{}", generation).unwrap();
    Text::new(
        num_str.as_str(),
        Point::new(8, 13),
        MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE),
    )
    .draw(display)?;
    Ok(())
}
const LCD_H_RES_USIZE: usize = 320;
const LCD_V_RES_USIZE: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

// Embassy multicore: allocate app core stack
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

static PSRAM_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static mut PSRAM_BUF_PTR: *mut u8 = core::ptr::null_mut();
static mut PSRAM_BUF_LEN: usize = 0;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    // Initialize TimerGroup and timer1 for app core Embassy time driver
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let timer1: AnyTimer = timg1.timer0.into();
    println!("Starting up...");
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    println!("PSRAM allocator initialized");

    init_logger_from_env();
    // esp_alloc::heap_allocator!(size: 72 * 1024);

    // Read display dimensions from EEPROM
    let i2c_bus = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO1)
        .with_scl(peripherals.GPIO41);
    let mut eeid = [0u8; 0x1c];
    let mut eeprom = Eeprom24x::new_24x01(i2c_bus, SlaveAddr::default());
    eeprom.read_data(0x00, &mut eeid).unwrap();
    let display_width = u16::from_be_bytes([eeid[8], eeid[9]]) as usize;
    let display_height = u16::from_be_bytes([eeid[10], eeid[11]]) as usize;
    info!(
        "Display size from EEPROM: {}x{}",
        display_width, display_height
    );

    // Full-screen DMA constants
    const MAX_FRAME_BYTES: usize = 320 * 240 * 2;
    const MAX_NUM_DMA_DESC: usize = (MAX_FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

    #[unsafe(link_section = ".dma")]
    static mut TX_DESCRIPTORS: [DmaDescriptor; MAX_NUM_DMA_DESC] =
        [DmaDescriptor::EMPTY; MAX_NUM_DMA_DESC];

    // Allocate framebuffer in PSRAM and reuse as DMA buffer
    const FRAME_BYTES: usize = LCD_BUFFER_SIZE * 2;
    let mut fb_box: Box<[Rgb565; LCD_BUFFER_SIZE]> = Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
    let fb_ptr: *mut Rgb565 = fb_box.as_mut_ptr();
    let psram_buf: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(fb_ptr as *mut u8, FRAME_BYTES) };
    // Verify PSRAM buffer allocation and alignment
    let buf_ptr = psram_buf.as_ptr() as usize;
    info!("PSRAM buffer allocated at address: 0x{:08X}", buf_ptr);
    info!("PSRAM buffer length: {}", psram_buf.len());
    info!("PSRAM buffer alignment modulo 32: {}", buf_ptr % 32);
    assert!(
        buf_ptr % 64 == 0,
        "PSRAM buffer must be 64-byte aligned for DMA"
    );
    // Publish PSRAM buffer pointer and len for app core
    unsafe {
        PSRAM_BUF_PTR = psram_buf.as_mut_ptr();
        PSRAM_BUF_LEN = psram_buf.len();
    }

    info!("Initializing display...");

    // Panel‐enable / backlight
    let mut panel_enable = Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default());
    // some boards require LOW→HIGH pulse, or just HIGH
    panel_enable.set_high();

    // Backlight enable (PWM output pin)
    let mut backlight = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default());
    backlight.set_high();

    // Touch reset line
    let mut _touch_reset = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    // Initialize the parallel‐RGB display via DPI
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let pclk_hz = ((eeid[12] as u32) * 1_000_000 + (eeid[13] as u32) * 100_000).min(13_600_000);
    let h_res = display_width;
    let v_res = display_height;
    let hsync_pulse = eeid[17] as u32;
    let hsync_back = u16::from_be_bytes([eeid[15], eeid[16]]) as u32;
    let hsync_front = u16::from_be_bytes([eeid[18], eeid[19]]) as u32;
    let vsync_pulse = eeid[22] as usize;
    let vsync_back = u16::from_be_bytes([eeid[20], eeid[21]]);
    let vsync_front = u16::from_be_bytes([eeid[23], eeid[24]]) as u32;

    // Read idle/polarity flags from EEPROM (eeid[25])
    let flags = eeid[25];
    let hsync_idle_low = (flags & 0x01) != 0;
    let vsync_idle_low = (flags & 0x02) != 0;
    let de_idle_high = (flags & 0x04) != 0;
    let pclk_active_neg = (flags & 0x20) != 0;

    // Log all infomration about display configuration
    info!("Display configuration:");
    info!("  Resolution: {}x{}", h_res, v_res);
    info!("  PCLK: {} Hz", pclk_hz);
    info!("  HSYNC pulse: {} pixels", hsync_pulse);
    info!("  HSYNC back porch: {} pixels", hsync_back);
    info!("  HSYNC front porch: {} pixels", hsync_front);
    info!("  VSYNC pulse: {} lines", vsync_pulse);
    info!("  VSYNC back porch: {} lines", vsync_back);
    info!("  VSYNC front porch: {} lines", vsync_front);

    let dpi_config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: if pclk_active_neg {
                Polarity::IdleHigh
            } else {
                Polarity::IdleLow
            },
            phase: if pclk_active_neg {
                Phase::ShiftHigh
            } else {
                Phase::ShiftLow
            },
        })
        .with_frequency(Rate::from_hz(pclk_hz))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        // The values read from EEPROM seems to differ from a working configuration
        .with_timing(FrameTiming {
            horizontal_active_width: 320,
            horizontal_total_width: 320 + 4 + 43 + 79 + 8, // =446
            horizontal_blank_front_porch: 79 + 8,          // was 47, add 32px
            vertical_active_height: 240,
            vertical_total_height: 240 + 4 + 12 + 16, // increased blank front porch to 16
            vertical_blank_front_porch: 16,
            hsync_width: 4,
            vsync_width: 4,
            hsync_position: 43 + 4, // (= back_porch + pulse = 47)
        })
        // apply idle levels based on EEPROM flags
        .with_vsync_idle_level(if vsync_idle_low {
            Level::Low
        } else {
            Level::High
        })
        .with_hsync_idle_level(if hsync_idle_low {
            Level::Low
        } else {
            Level::High
        })
        .with_de_idle_level(if de_idle_high {
            Level::High
        } else {
            Level::Low
        })
        .with_disable_black_region(false);

    let dpi = Dpi::new(lcd_cam.lcd, peripherals.DMA_CH2, dpi_config)
        .unwrap()
        .with_vsync(peripherals.GPIO6)
        .with_hsync(peripherals.GPIO15)
        .with_de(peripherals.GPIO5)
        .with_pclk(peripherals.GPIO4)
        // Blue bus
        .with_data0(peripherals.GPIO9)
        .with_data1(peripherals.GPIO17)
        .with_data2(peripherals.GPIO46)
        .with_data3(peripherals.GPIO16)
        .with_data4(peripherals.GPIO7)
        // Green bus
        .with_data5(peripherals.GPIO8)
        .with_data6(peripherals.GPIO21)
        .with_data7(peripherals.GPIO3)
        .with_data8(peripherals.GPIO11)
        .with_data9(peripherals.GPIO18)
        .with_data10(peripherals.GPIO10)
        // Red bus
        .with_data11(peripherals.GPIO14)
        .with_data12(peripherals.GPIO20)
        .with_data13(peripherals.GPIO13)
        .with_data14(peripherals.GPIO19)
        .with_data15(peripherals.GPIO12);

    // Prepare RNG for app core task
    let rng_for_app = Rng::new(peripherals.RNG);

    // The PSRAM framebuffer is now used for drawing in the Conway task.
    info!("Entering main loop...");

    // Configure a single DMA buffer over the whole PSRAM region with 64‑byte bursts
    let dma_tx: DmaTxBuf = unsafe {
        DmaTxBuf::new_with_config(&mut *core::ptr::addr_of_mut!(TX_DESCRIPTORS), psram_buf, ExternalBurstConfig::Size64)
            .unwrap()
    };

    // Signal to app core that PSRAM is ready
    PSRAM_READY.signal(());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();

    // Spawn DMA display task on app core (core 1)
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let _app_core = cpu_control.start_app_core(unsafe { &mut *core::ptr::addr_of_mut!(APP_CORE_STACK) }, move || {
        // Initialize Embassy time driver on app core
        esp_hal_embassy::init([timer0, timer1]);
        info!("[CORE 1] App core started, initializing DMA display task");
        // Initialize and run Embassy executor on app core
        static EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = EXECUTOR.init(Executor::new());
        executor.run(|spawner| {
            spawner
                .spawn(dma_display_task(dpi, dma_tx))
                .ok();
        });
    });

    // Core 0: Run Conway task
    // Wait until PSRAM is ready
    loop {
        if PSRAM_READY.try_take().is_some() {
            break;
        }
        // Simple spin wait
        core::hint::spin_loop();
    }
    // SAFETY: PSRAM_BUF_PTR and PSRAM_BUF_LEN are published before
    let psram_ptr = unsafe { PSRAM_BUF_PTR };
    let psram_len = unsafe { PSRAM_BUF_LEN };

    static MAIN_EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = MAIN_EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(conway_task(psram_ptr, psram_len, rng_for_app)).ok();
    });
}

// DMA display task running on core 1
#[embassy_executor::task]
async fn dma_display_task(mut dpi: Dpi<'static, esp_hal::Blocking>, mut dma_tx: DmaTxBuf) {
    info!("[CORE 1] DMA display task started, sending DMA frames");

    loop {
        let safe_chunk_size = 320 * 240 * 2;
        let frame_bytes = 320 * 240 * 2; // Fixed to known display size
        let len = safe_chunk_size.min(frame_bytes);
        dma_tx.set_length(len);

        match dpi.send(false, dma_tx) {
            Ok(xfer) => {
                let (res, new_dpi, new_dma_tx) = xfer.wait();
                dpi = new_dpi;
                dma_tx = new_dma_tx;
                if let Err(e) = res {
                    error!("[CORE 1] DMA transfer error: {:?}", e);
                }
            }
            Err((e, new_dpi, new_dma_tx)) => {
                error!("[CORE 1] DMA send error: {:?}", e);
                dpi = new_dpi;
                dma_tx = new_dma_tx;
            }
        }
    }
}

// Conway update task running on core 0
#[embassy_executor::task]
async fn conway_task(psram_ptr: *mut u8, _psram_len: usize, mut rng: Rng) {
    // Reconstruct the framebuffer and game grid
    let fb: &mut [Rgb565; LCD_BUFFER_SIZE] =
        unsafe { &mut *(psram_ptr as *mut [Rgb565; LCD_BUFFER_SIZE]) };
    let mut game_grid = Box::new([[0u8; GRID_WIDTH]; GRID_HEIGHT]);
    randomize_grid(&mut rng, &mut *game_grid);
    let mut next_grid = Box::new([[0u8; GRID_WIDTH]; GRID_HEIGHT]);

    let mut frame_buf = FrameBuf::new(
        PSRAMFrameBuffer::new(fb),
        LCD_H_RES_USIZE.into(),
        LCD_V_RES_USIZE.into(),
    );
    let mut ticker = Ticker::every(embassy_time::Duration::from_millis(100));
    let mut generation_count: usize = 0;
    const RESET_AFTER_GENERATIONS: usize = 500;
    loop {
        // Log message with core number
        // println!(
        //     "Core {}: Updating Game of Life grid...",
        //     Cpu::current() as usize
        // );

        update_game_of_life(&*game_grid, &mut *next_grid);
        core::mem::swap(&mut game_grid, &mut next_grid);
        draw_grid(&mut frame_buf, &*game_grid).ok();
        generation_count += 1;
        write_generation(&mut frame_buf, generation_count).ok();
        if generation_count >= RESET_AFTER_GENERATIONS {
            randomize_grid(&mut rng, &mut *game_grid);
            generation_count = 0;
        }
        ticker.next().await;
    }
}
