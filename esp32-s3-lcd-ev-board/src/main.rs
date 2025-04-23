#![no_std]
#![no_main]

extern crate alloc;

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
};
use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    main,
};
use esp_hal::clock::CpuClock::_240MHz;
use esp_println::{logger::init_logger_from_env, println};
use log::{info, error};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("Panic: {}", info);
    loop {}
}

// ST7262E43 Display resolution
const LCD_H_RES: u16 = 800;
const LCD_V_RES: u16 = 480;

// RGB timing parameters for 35Hz refresh rate
// These values need to be adjusted based on the actual panel timing requirements
const HSYNC_PULSE_WIDTH: u32 = 48;     // HSYNC pulse width in PCLK cycles
const HSYNC_BACK_PORCH: u32 = 88;      // HSYNC back porch in PCLK cycles
const HSYNC_FRONT_PORCH: u32 = 40;     // HSYNC front porch in PCLK cycles
const VSYNC_PULSE_WIDTH: u32 = 3;      // VSYNC pulse width in lines
const VSYNC_BACK_PORCH: u32 = 32;      // VSYNC back porch in lines
const VSYNC_FRONT_PORCH: u32 = 13;     // VSYNC front porch in lines
const PCLK_ACTIVE_NEG: bool = true;    // True if pixel clock is active on negative edge
const PCLK_DIVIDER: u32 = 4;


// RGB 16-bit parallel interface type
struct RGB16BitInterface {
    de: Output<'static, >,       // Data enable pin
    pclk: Output<'static, >,     // Pixel clock pin
    vsync: Output<'static, >,    // Vertical sync pin
    hsync: Output<'static, >,    // Horizontal sync pin
    data_pins: [Output<'static, >; 16], // 16 data pins
}

#[inline(always)]
fn delay_cycles(cycles: u32) {
    // A simple software delay loop
    let mut remaining = cycles;
    while remaining > 0 {
        // Compiler barrier to prevent optimization
        core::hint::spin_loop();
        remaining -= 1;
    }
}


impl RGB16BitInterface {
    // Create a new RGB interface with the provided pins
    pub fn new(
        de: Output<'static, >,
        pclk: Output<'static, >,
        vsync: Output<'static, >,
        hsync: Output<'static, >,
        data_pins: [Output<'static>; 16],
    ) -> Self {
        Self {
            de,
            pclk,
            vsync,
            hsync,
            data_pins,
        }
    }

    // Initialize the RGB interface
    pub fn init(&mut self) -> Result<(), ()> {
        info!("Initializing RGB interface for ST7262E43");

        // Initialize all signals to their inactive state
        self.de.set_low();
        self.pclk.set_low();
        self.vsync.set_high();
        self.hsync.set_high();

        // Set all data pins to low
        for pin in &mut self.data_pins {
            pin.set_low();
        }

        // Wait for power stabilization
        let mut delay = Delay::new();
        delay.delay_ms(50);

        // Reset sequence - pulse VSYNC several times
        // Some displays need a specific sequence at startup
        for _ in 0..5 {
            self.vsync.set_low();
            delay.delay_ms(1);
            self.vsync.set_high();
            delay.delay_ms(1);
        }

        // Additional delay for display to initialize
        delay.delay_ms(100);

        info!("RGB interface initialized");
        Ok(())
    }

    // Begin a new frame with precise timing
    pub fn start_frame(&mut self) -> Result<(), ()> {
        // VSYNC pulse to start a new frame
        self.vsync.set_low();

        // VSYNC pulse width
        for _ in 0..VSYNC_PULSE_WIDTH {
            // One complete line timing
            self.hsync.set_low();
            delay_cycles(HSYNC_PULSE_WIDTH * 10); // Scale delay as needed
            self.hsync.set_high();

            // Remaining line time (back porch + active + front porch)
            delay_cycles((HSYNC_BACK_PORCH + LCD_H_RES as u32 + HSYNC_FRONT_PORCH) * 10);
        }

        self.vsync.set_high();

        // VSYNC back porch
        for _ in 0..VSYNC_BACK_PORCH {
            // One complete line timing
            self.hsync.set_low();
            delay_cycles(HSYNC_PULSE_WIDTH * 10);
            self.hsync.set_high();

            // Remaining line time
            delay_cycles((HSYNC_BACK_PORCH + LCD_H_RES as u32 + HSYNC_FRONT_PORCH) * 10);
        }

        Ok(())
    }

    // Begin a new line with precise timing
    pub fn start_line(&mut self) -> Result<(), ()> {
        // HSYNC pulse to start a new line
        self.hsync.set_low();

        // HSYNC pulse width with precise timing
        delay_cycles(HSYNC_PULSE_WIDTH * 10);

        self.hsync.set_high();

        // HSYNC back porch with precise timing
        delay_cycles(HSYNC_BACK_PORCH * 10);

        // Set DE high to indicate active pixel data
        self.de.set_high();

        Ok(())
    }

    // End the current line with precise timing
    pub fn end_line(&mut self) -> Result<(), ()> {
        // Set DE low to indicate end of active pixel data
        self.de.set_low();

        // HSYNC front porch with precise timing
        delay_cycles(HSYNC_FRONT_PORCH * 10);

        Ok(())
    }

    pub fn end_frame(&mut self) -> Result<(), ()> {
        // End of frame - set DE low
        self.de.set_low();

        // Additional delay for frame end
        delay_cycles(1000);

        Ok(())
    }

    // Send a single pixel of RGB565 color data
    pub fn send_pixel(&mut self, color: Rgb565) -> Result<(), ()> {
        // Extract color components
        let color_value = color.into_storage();

        // Set 16 data pins according to RGB565 format
        for bit in 0..16 {
            if (color_value & (1 << bit)) != 0 {
                self.data_pins[bit].set_high();
            } else {
                self.data_pins[bit].set_low();
            }
        }

        // Pulse PCLK to latch the data
        let _ = self.pulse_pclk();

        Ok(())
    }

    // Helper to pulse the pixel clock
    fn pulse_pclk(&mut self) -> Result<(), ()> {
        if PCLK_ACTIVE_NEG {
            self.pclk.set_high();
            delay_cycles(PCLK_DIVIDER);
            self.pclk.set_low();
            delay_cycles(PCLK_DIVIDER);
        } else {
            self.pclk.set_low();
            delay_cycles(PCLK_DIVIDER);
            self.pclk.set_high();
            delay_cycles(PCLK_DIVIDER);
        }
        Ok(())
    }
}

// Display controller for ST7262E43
struct ST7262E43Display {
    rgb_interface: RGB16BitInterface,
}

impl ST7262E43Display {
    pub fn new(rgb_interface: RGB16BitInterface) -> Self {
        Self { rgb_interface }
    }

    pub fn init(&mut self) -> Result<(), ()> {
        // Initialize the RGB interface
        self.rgb_interface.init()?;

        info!("ST7262E43 display initialized");
        Ok(())
    }

    pub fn clear(&mut self, color: Rgb565) -> Result<(), ()> {
        info!("Clearing display with color: {:?}", color);

        self.rgb_interface.start_frame()?;

        for y in 0..LCD_V_RES {
            self.rgb_interface.start_line()?;

            for _ in 0..LCD_H_RES {
                self.rgb_interface.send_pixel(color)?;
            }

            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }

    pub fn draw_test_pattern(&mut self) -> Result<(), ()> {
        info!("Drawing simple test pattern");

        self.rgb_interface.start_frame()?;

        for y in 0..LCD_V_RES {
            self.rgb_interface.start_line()?;

            // Draw alternating bands of color
            let color = match (y / 40) % 4 {
                0 => Rgb565::RED,
                1 => Rgb565::GREEN,
                2 => Rgb565::BLUE,
                _ => Rgb565::WHITE,
            };

            // For each line, fill with the same color
            for _ in 0..LCD_H_RES {
                self.rgb_interface.send_pixel(color)?;
            }

            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }

    pub fn draw_gradient(&mut self) -> Result<(), ()> {
        info!("Drawing gradient");

        self.rgb_interface.start_frame()?;

        for y in 0..LCD_V_RES {
            self.rgb_interface.start_line()?;

            for x in 0..LCD_H_RES {
                // Create a gradient color
                let r = (x * 31 / LCD_H_RES) as u8;
                let g = (y * 63 / LCD_V_RES) as u8;
                let b = 31 - (x * 31 / LCD_H_RES) as u8;

                let color = Rgb565::new(r, g, b);
                self.rgb_interface.send_pixel(color)?;
            }

            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }

    pub fn draw_checkerboard(&mut self, size: u16) -> Result<(), ()> {
        info!("Drawing checkerboard with square size: {}", size);

        self.rgb_interface.start_frame()?;

        for y in 0..LCD_V_RES {
            self.rgb_interface.start_line()?;

            for x in 0..LCD_H_RES {
                let is_white = ((x / size) + (y / size)) % 2 == 0;
                let color = if is_white { Rgb565::WHITE } else { Rgb565::BLACK };

                self.rgb_interface.send_pixel(color)?;
            }

            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }
}

#[main]
fn main() -> ! {
    // Initialize the ESP32-S3
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(_240MHz));

    let mut pll_delay = Delay::new();
    pll_delay.delay_ms(100);


    esp_alloc::heap_allocator!(size: 32 * 1024);
    init_logger_from_env();

    // Create the delay provider
    let mut delay = Delay::new();

    // Initialize GPIO pins for RGB interface
    info!("Initializing GPIO pins for RGB interface");

    // Create pins with correct drive strength for LCD

    // Data pins (adjust pin numbers as needed for your board)
    let data_pins = [
        Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default()), // DATA0
        Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default()), // DATA1
        Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default()), // DATA2
        Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default()), // DATA3
        Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default()), // DATA4
        Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default()), // DATA5
        Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default()), // DATA6
        Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default()), // DATA7
        Output::new(peripherals.GPIO45, Level::Low, OutputConfig::default()), // DATA8
        Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default()), // DATA9
        Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default()), // DATA10
        Output::new(peripherals.GPIO40, Level::Low, OutputConfig::default()), // DATA11
        Output::new(peripherals.GPIO41, Level::Low, OutputConfig::default()), // DATA12
        Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default()), // DATA13
        Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default()),  // DATA14
        Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default()),  // DATA15

    ];

    // Control signals
    let hsync = Output::new(peripherals.GPIO46, Level::High, OutputConfig::default());  // HSYNC
    let vsync = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());   // VSYNC
    let de = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());      // DE
    let pclk = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());     // PCLK

    // Create RGB interface
    let rgb_interface = RGB16BitInterface::new(de, pclk, vsync, hsync, data_pins);

    // Create and initialize the display controller
    let mut display = ST7262E43Display::new(rgb_interface);
    if let Err(()) = display.init() {
        error!("Failed to initialize display");
        panic!("Display initialization failed");
    }

    // Wait a moment for initialization to complete
    delay.delay_ms(100);

    // Test display with various patterns
    info!("Starting display tests");

    // Clear with solid colors
    if let Err(()) = display.clear(Rgb565::RED) {
        error!("Failed to clear display with red");
    }
    delay.delay_ms(1000);

    if let Err(()) = display.clear(Rgb565::GREEN) {
        error!("Failed to clear display with green");
    }
    delay.delay_ms(1000);

    if let Err(()) = display.clear(Rgb565::BLUE) {
        error!("Failed to clear display with blue");
    }
    delay.delay_ms(1000);

    if let Err(()) = display.clear(Rgb565::WHITE) {
        error!("Failed to clear display with white");
    }
    delay.delay_ms(1000);

    // Draw test patterns
    if let Err(()) = display.draw_test_pattern() {
        error!("Failed to draw test pattern");
    }
    delay.delay_ms(2000);

    if let Err(()) = display.draw_gradient() {
        error!("Failed to draw gradient");
    }
    delay.delay_ms(2000);

    if let Err(()) = display.draw_checkerboard(40) {
        error!("Failed to draw checkerboard");
    }
    delay.delay_ms(2000);

    // Cycle through test patterns
    info!("Entering main loop");

    let mut count = 0;
    loop {
        match count % 4 {
            0 => {
                if let Err(()) = display.draw_test_pattern() {
                    error!("Failed to draw test pattern");
                }
            },
            1 => {
                if let Err(()) = display.draw_gradient() {
                    error!("Failed to draw gradient");
                }
            },
            2 => {
                if let Err(()) = display.draw_checkerboard(40) {
                    error!("Failed to draw checkerboard");
                }
            },
            _ => {
                if let Err(()) = display.clear(Rgb565::new(
                    (count % 32) as u8,
                    ((count * 2) % 64) as u8,
                    ((32 - count) % 32) as u8
                )) {
                    error!("Failed to clear display with color");
                }
            }
        }

        count += 1;
        delay.delay_ms(2000);
    }
}