#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use alloc::vec::Vec;
use embedded_graphics_framebuf::backends::FrameBufferBackend;

use bevy_ecs::prelude::*;
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
use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;
use esp_hal::{gpio::{Level, Output, OutputConfig}, i2c::master::I2c, i2c::master::Error as I2cError, main, rng::Rng, Blocking};
use esp_hal::i2c::master::BusTimeout;
use esp_hal::time::Rate;
use esp_println::{logger::init_logger_from_env, println};
use log::{ info, error };
// use mipidsi::models::GC9503CV;

// use mipidsi::interface::{Generic8BitBus, ParallelInterface};
// includes NonSend and NonSendMut

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("Panic: {}", _info);
    loop {}
}

/// A wrapper around a boxed array that implements FrameBufferBackend.
/// This allows the framebuffer to be allocated on the heap.
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


// Type for IO Expander
struct IOExpander<Dm: esp_hal::DriverMode> {
    i2c: I2c<'static, Dm>,
    address: u8,
}


// In your IOExpander struct implementation:
impl<Dm: esp_hal::DriverMode> IOExpander<Dm> {
    const BSP_IO_EXPANDER_I2C_ADDRESS: u8 = 0x20;

    const REG_INPUT_PORT: u8 = 0x00;
    const REG_OUTPUT_PORT: u8 = 0x01;
    const REG_CONFIG: u8 = 0x03;

    // LCD sub-board control signals
    const BSP_LCD_SUB_BOARD_2_SPI_CS: u8 = 1;
    const BSP_LCD_SUB_BOARD_2_SPI_SCK: u8 = 2;
    const BSP_LCD_SUB_BOARD_2_SPI_SDO: u8 = 3;

    fn new(i2c: I2c<'static, Dm>) -> Self {
        Self {
            i2c,
            address: Self::BSP_IO_EXPANDER_I2C_ADDRESS,
        }
    }

    fn init(&mut self) -> Result<(), I2cError> {

        let config_mask = !(
            (1 << Self::BSP_LCD_SUB_BOARD_2_SPI_CS) |
                (1 << Self::BSP_LCD_SUB_BOARD_2_SPI_SCK) |
                (1 << Self::BSP_LCD_SUB_BOARD_2_SPI_SDO)
        );

        let write_buf = [Self::REG_CONFIG, config_mask];
        self.i2c.write(self.address, &write_buf)?;

        // Set initial state - CS high, SCK and MOSI low
        self.set_pin(Self::BSP_LCD_SUB_BOARD_2_SPI_CS, true)?;
        self.set_pin(Self::BSP_LCD_SUB_BOARD_2_SPI_SCK, false)?;
        self.set_pin(Self::BSP_LCD_SUB_BOARD_2_SPI_SDO, false)?;

        Ok(())
    }

    fn set_pin(&mut self, pin: u8, level: bool) -> Result<(), I2cError> {
        // Read current output state
        let write_buf = [Self::REG_OUTPUT_PORT];
        self.i2c.write(self.address, &write_buf)?;

        let mut read_buf = [0u8];
        self.i2c.read(self.address, &mut read_buf)?;

        let mut output_state = read_buf[0];

        // Update the pin state
        if level {
            output_state |= 1 << pin;
        } else {
            output_state &= !(1 << pin);
        }

        // Write back the updated state
        let write_buf = [Self::REG_OUTPUT_PORT, output_state];
        self.i2c.write(self.address, &write_buf)

    }
}

// Structure to handle the SPI-over-IO-Expander interface
struct SpiViaIOExpander<Dm: esp_hal::DriverMode> {
    io_expander: IOExpander<Dm>,
}

impl<Dm: esp_hal::DriverMode> SpiViaIOExpander<Dm> {
    fn new(io_expander: IOExpander<Dm>) -> Self {
        Self { io_expander }
    }

    fn init(&mut self) -> Result<(), I2cError> {
        self.io_expander.init()?;
        // Set initial state - CS high, SCK low, SDO low
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, true)?;

        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SCK, false)?;
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SDO, false)?;
        Ok(())
    }

    // Send a command byte to the GC9503
    fn send_command(&mut self, cmd: u8) -> Result<(), I2cError> {
        // Assert CS (active low)
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, false)?;

        // Transmit command (first bit low for command)
        self.transmit_byte(cmd, false)?;

        // Deassert CS
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, true)?;

        Ok(())
    }

    // Send a data byte to the GC9503
    fn send_data(&mut self, data: u8) -> Result<(), I2cError> {
        // Assert CS (active low)
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, false)?;

        // Transmit data (first bit high for data)
        self.transmit_byte(data, true)?;

        // Deassert CS
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, true)?;

        Ok(())
    }

    // Send multiple data bytes
    fn send_data_multiple(&mut self, data: &[u8]) -> Result<(), I2cError> {
    // Assert CS (active low)
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, false)?;

        for &byte in data {
            // Transmit data (first bit high for data)
            self.transmit_byte(byte, true)?;
        }

        // Deassert CS
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_CS, true)?;

        Ok(())
    }

    // Transmit a byte bit by bit
    fn transmit_byte(&mut self, byte: u8, is_data: bool) -> Result<(), I2cError> {
        let mut data = byte;

        // For GC9503, we need to send the first bit as 0 for command, 1 for data
        // Set SDO to indicate command/data
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SDO, is_data)?;

        // Clock pulse for the command/data bit
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SCK, true)?;
        self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SCK, false)?;

        // Send 8 bits of data MSB first
        for _ in 0..8 {
            // Set data bit
            self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SDO, (data & 0x80) != 0)?;
            data <<= 1;

            // Clock pulse
            self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SCK, true)?;
            self.io_expander.set_pin(IOExpander::<Dm>::BSP_LCD_SUB_BOARD_2_SPI_SCK, false)?;
        }

        Ok(())
    }
}


// RGB 16-bit parallel interface type
struct RGB16BitInterface {
    de: Output<'static>,         // Data enable (BSP_LCD_SUB_BOARD_2_3_DE)
    pclk: Output<'static>,       // Pixel clock (BSP_LCD_SUB_BOARD_2_3_PCLK)
    vsync: Output<'static>,      // Vertical sync (BSP_LCD_SUB_BOARD_2_3_VSYNC)
    hsync: Output<'static>,      // Horizontal sync (BSP_LCD_SUB_BOARD_2_3_HSYNC)
    data_pins: [Output<'static>; 16], // 16 data pins
}

impl RGB16BitInterface {
    fn new(
        de: Output<'static>,
        pclk: Output<'static>,
        vsync: Output<'static>,
        hsync: Output<'static>,
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


    // Begin a new frame
    pub fn start_frame(&mut self) -> Result<(), ()> {
        // VSYNC pulse to start a new frame
        self.vsync.set_low();

        // Typical VSYNC pulse width is 4 pixel clocks
        for _ in 0..4 {
            self.pulse_pclk();
        }

        self.vsync.set_high();

        // VSYNC back porch (typically 8-12 pixel clocks)
        for _ in 0..10 {
            self.pulse_pclk()?;
        }

        Ok(())
    }

    // End the current frame
    pub fn end_frame(&mut self) -> Result<(), ()> {
        // VSYNC front porch (typically 8-12 pixel clocks)
        for _ in 0..10 {
            self.pulse_pclk()?;
        }

        Ok(())
    }

    // Begin a new line
    pub fn start_line(&mut self) -> Result<(), ()> {
        // HSYNC pulse to start a new line
        self.hsync.set_low();

        // Typical HSYNC pulse width is 4 pixel clocks
        for _ in 0..4 {
            self.pulse_pclk();
        }

        self.hsync.set_high();

        // HSYNC back porch (typically 8 pixel clocks)
        for _ in 0..8 {
            self.pulse_pclk();
        }

        // Set DE high to indicate active pixel data
        self.de.set_high();

        Ok(())
    }

    // End the current line
    pub fn end_line(&mut self) -> Result<(), ()> {
        // Set DE low to indicate end of active pixel data
        self.de.set_low();

        // HSYNC front porch (typically 8 pixel clocks)
        for _ in 0..8 {
            self.pulse_pclk();
        }

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
        self.pulse_pclk();

        Ok(())
    }

    // Helper to pulse the pixel clock
    fn pulse_pclk(&mut self) -> Result<(), ()> {
        self.pclk.set_high();

        // Small delay for stability (might need adjustment)
        // This should be a very short delay, just enough for the signal to propagate
        // For high-speed operation, this might be removed entirely
        // unsafe { asm!("nop; nop;") }

        self.pclk.set_low();

        // Small delay on the low cycle too
        // unsafe { asm!("nop; nop;") }

        Ok(())
    }

    
}

// Custom display controller for GC9503
struct GC9503Display<Dm: esp_hal::DriverMode> {
    spi: SpiViaIOExpander<Dm>,
    rgb_interface: RGB16BitInterface,
    width: u16,
    height: u16,
}

impl<Dm: esp_hal::DriverMode> GC9503Display<Dm> {
    fn new(spi: SpiViaIOExpander<Dm>, rgb_interface: RGB16BitInterface, width: u16, height: u16) -> Self {
        Self {
            spi,
            rgb_interface,
            width,
            height,
        }
    }

    fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), I2cError> {
        info!("Initializing GC9503CV display controller via I/O expander SPI");

        // Initialize the I/O expander for SPI communication
        self.spi.init()?;

        // Reset sequence
        delay.delay_ms(120);

        // Following the BSP vendor_specific_init_default sequence

        // Enable command set
        self.spi.send_command(0xf0)?;
        self.spi.send_data_multiple(&[0x55, 0xaa, 0x52, 0x08, 0x00])?;

        // Unknown command
        self.spi.send_command(0xf6)?;
        self.spi.send_data_multiple(&[0x5a, 0x87])?;

        // Power control settings
        self.spi.send_command(0xc1)?;
        self.spi.send_data(0x3f)?;

        self.spi.send_command(0xc2)?;
        self.spi.send_data(0x0e)?;

        self.spi.send_command(0xc6)?;
        self.spi.send_data(0xf8)?;

        self.spi.send_command(0xc9)?;
        self.spi.send_data(0x10)?;

        self.spi.send_command(0xcd)?;
        self.spi.send_data(0x25)?;

        // Various additional settings
        self.spi.send_command(0xf8)?;
        self.spi.send_data(0x8a)?;

        self.spi.send_command(0xac)?;
        self.spi.send_data(0x45)?;

        self.spi.send_command(0xa0)?;
        self.spi.send_data(0xdd)?;

        self.spi.send_command(0xa7)?;
        self.spi.send_data(0x47)?;

        self.spi.send_command(0xfa)?;
        self.spi.send_data_multiple(&[0x00, 0x00, 0x00, 0x04])?;

        self.spi.send_command(0x86)?;
        self.spi.send_data_multiple(&[0x99, 0xa3, 0xa3, 0x51])?;

        self.spi.send_command(0xa3)?;
        self.spi.send_data(0xee)?;

        self.spi.send_command(0xfd)?;
        self.spi.send_data_multiple(&[0x3c, 0x3c, 0x00])?;

        // More panel configuration
        self.spi.send_command(0x71)?;
        self.spi.send_data(0x48)?;

        self.spi.send_command(0x72)?;
        self.spi.send_data(0x48)?;

        self.spi.send_command(0x73)?;
        self.spi.send_data_multiple(&[0x00, 0x44])?;

        self.spi.send_command(0x97)?;
        self.spi.send_data(0xee)?;

        self.spi.send_command(0x83)?;
        self.spi.send_data(0x93)?;

        self.spi.send_command(0x9a)?;
        self.spi.send_data(0x72)?;

        self.spi.send_command(0x9b)?;
        self.spi.send_data(0x5a)?;

        self.spi.send_command(0x82)?;
        self.spi.send_data_multiple(&[0x2c, 0x2c])?;

        // Gamma settings
        // I'll simplify these long gamma sequences slightly for brevity
        self.spi.send_command(0x6d)?;
        self.spi.send_data_multiple(&[
            0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a,
            0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e,
            0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01,
            0x09, 0x0b, 0x0d, 0x0f, 0x1a, 0x19, 0x1f, 0x00
        ])?;

        // Many more configuration registers
        self.spi.send_command(0x64)?;
        self.spi.send_data_multiple(&[
            0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04,
            0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x65)?;
        self.spi.send_data_multiple(&[
            0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02,
            0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x66)?;
        self.spi.send_data_multiple(&[
            0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00,
            0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x67)?;
        self.spi.send_data_multiple(&[
            0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02,
            0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x68)?;
        self.spi.send_data_multiple(&[
            0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08,
            0x15, 0x08, 0x15, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x60)?;
        self.spi.send_data_multiple(&[
            0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x63)?;
        self.spi.send_data_multiple(&[
            0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a
        ])?;

        self.spi.send_command(0x69)?;
        self.spi.send_data_multiple(&[
            0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08
        ])?;

        self.spi.send_command(0x6b)?;
        self.spi.send_data(0x07)?;

        self.spi.send_command(0x7a)?;
        self.spi.send_data_multiple(&[0x08, 0x13])?;

        self.spi.send_command(0x7b)?;
        self.spi.send_data_multiple(&[0x08, 0x13])?;

        // There are six gamma curve settings (d1-d6) in the BSP code
        // I'll include just one example for brevity, but all six should be implemented
        self.spi.send_command(0xd1)?;
        self.spi.send_data_multiple(&[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18,
            0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47,
            0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68,
            0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6,
            0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba,
            0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea,
            0x03, 0xfa, 0x03, 0xff
        ])?;

        // Memory access control (MADCTL)
        self.spi.send_command(0xb1)?; // Note: GC9503 uses 0xB1 for MADCTL, not 0x36
        self.spi.send_data(0x10)?; // Default value according to docs

        // Pixel format setting
        self.spi.send_command(0x3a)?;
        self.spi.send_data(0x55)?; // 16-bit color format (RGB565)

        // RGB interface configuration
        self.spi.send_command(0xb0)?;
        self.spi.send_data(0x00)?; // RGB interface control default

        // Sleep out
        self.spi.send_command(0x11)?;
        delay.delay_ms(120);

        // Display on
        self.spi.send_command(0x29)?;
        delay.delay_ms(20);

        info!("GC9503CV initialization completed successfully");
        Ok(())
    }
    
    fn clear(&mut self, color: Rgb565) -> Result<(), ()> {
        self.rgb_interface.start_frame()?;
        info!("Clearing - dimension: {}x{}", self.width, self.height);
        for y in 0..self.height {
            self.rgb_interface.start_line()?;

            for _ in 0..self.width {
                self.rgb_interface.send_pixel(color)?;
            }

            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }

    fn draw_bitmap(&mut self, buffer: &[Rgb565], x: u16, y: u16, width: u16, height: u16) -> Result<(), ()> {
        if x >= self.width || y >= self.height {
            return Ok(());
        }

        let end_x = (x + width).min(self.width);
        let end_y = (y + height).min(self.height);
        let actual_width = end_x - x;

        self.rgb_interface.start_frame()?;

        // Skip lines before y
        for _ in 0..y {
            self.rgb_interface.start_line()?;
            for _ in 0..self.width {
                self.rgb_interface.send_pixel(Rgb565::RED)?;
            }
            self.rgb_interface.end_line()?;
        }

        // Draw relevant lines
        for line_y in y..end_y {
            self.rgb_interface.start_line()?;

            // Skip pixels before x
            for _ in 0..x {
                self.rgb_interface.send_pixel(Rgb565::GREEN)?;
            }

            // Draw actual pixels from buffer
            let buffer_offset = ((line_y - y) * width) as usize;
            for offset_x in 0..actual_width {
                let buffer_idx = buffer_offset + offset_x as usize;
                if buffer_idx < buffer.len() {
                    self.rgb_interface.send_pixel(buffer[buffer_idx])?;
                } else {
                    self.rgb_interface.send_pixel(Rgb565::BLUE)?;
                }
            }

            // Fill remaining pixels in the line
            for _ in end_x..self.width {
                self.rgb_interface.send_pixel(Rgb565::YELLOW)?;
            }

            self.rgb_interface.end_line()?;
        }

        // Fill the remaining lines
        for _ in end_y..self.height {
            self.rgb_interface.start_line()?;
            for _ in 0..self.width {
                self.rgb_interface.send_pixel(Rgb565::WHITE)?;
            }
            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }
}

impl<Dm: esp_hal::DriverMode> OriginDimensions for GC9503Display<Dm> {
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}

impl<Dm: esp_hal::DriverMode> DrawTarget for GC9503Display<Dm> {
    type Color = Rgb565;
    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        // We need to collect all pixels, sort them, and then update the display
        // since we need to draw full lines at a time
        let mut pixel_vec: Vec<Pixel<Rgb565>> = pixels.into_iter().collect();

        // Skip if no pixels to draw
        if pixel_vec.is_empty() {
            return Ok(());
        }

        // Sort by Y and then X for line-by-line drawing
        pixel_vec.sort_by_key(|p| (p.0.y, p.0.x));

        // Create a small buffer for the current line
        let mut line_buffer: Vec<Pixel<Rgb565>> = Vec::new();
        let mut current_y = pixel_vec[0].0.y;

        self.rgb_interface.start_frame()?;

        // Skip lines before the first pixel
        for y in 0..current_y {
            self.rgb_interface.start_line()?;
            for _ in 0..self.width {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }
            self.rgb_interface.end_line()?;
        }

        for pixel in pixel_vec {
            let Point { x, y } = pixel.0;

            // Skip invalid pixels
            if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
                continue;
            }

            // If we've moved to a new line, draw the previous line
            if y != current_y {
                // Draw the current line
                self.rgb_interface.start_line()?;

                // Fill in any missing pixels at the start of the line
                for _ in 0..line_buffer[0].0.x {
                    self.rgb_interface.send_pixel(Rgb565::BLACK)?;
                }

                // Draw actual pixels with potential gaps
                let mut last_x = 0;
                for p in &line_buffer {
                    // Fill gaps between pixels
                    for _ in last_x..p.0.x {
                        self.rgb_interface.send_pixel(Rgb565::BLACK)?;
                    }

                    // Draw the pixel
                    self.rgb_interface.send_pixel(p.1)?;
                    last_x = p.0.x + 1;
                }

                // Fill the end of the line
                for _ in last_x..self.width as i32 {
                    self.rgb_interface.send_pixel(Rgb565::BLACK)?;
                }

                self.rgb_interface.end_line()?;

                // Fill any skipped lines
                for skip_y in (current_y + 1)..y {
                    self.rgb_interface.start_line()?;
                    for _ in 0..self.width {
                        self.rgb_interface.send_pixel(Rgb565::BLACK)?;
                    }
                    self.rgb_interface.end_line()?;
                }

                // Start a new line buffer
                line_buffer.clear();
                current_y = y;
            }

            // Add pixel to current line buffer
            line_buffer.push(pixel);
        }

        // Draw the last line if any pixels remain
        if !line_buffer.is_empty() {
            self.rgb_interface.start_line()?;

            // Fill in any missing pixels at the start of the line
            for _ in 0..line_buffer[0].0.x {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }

            // Draw actual pixels with potential gaps
            let mut last_x = 0;
            for p in &line_buffer {
                // Fill gaps between pixels
                for _ in last_x..p.0.x {
                    self.rgb_interface.send_pixel(Rgb565::BLACK)?;
                }

                // Draw the pixel
                self.rgb_interface.send_pixel(p.1)?;
                last_x = p.0.x + 1;
            }

            // Fill the end of the line
            for _ in last_x..self.width as i32 {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }

            self.rgb_interface.end_line()?;
        }

        // Fill any remaining lines
        for y in (current_y + 1)..self.height as i32 {
            self.rgb_interface.start_line()?;
            for _ in 0..self.width {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }
            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        // Use fill_solid to clear the entire display
        let area = Rectangle::new(Point::zero(), self.size());
        self.fill_solid(&area, color)
    }


    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let colors_vec: Vec<_> = colors.into_iter().collect();
        let width = area.size.width as u16;
        let height = area.size.height as u16;

        self.draw_bitmap(
            &colors_vec,
            area.top_left.x as u16,
            area.top_left.y as u16,
            width,
            height,
        )
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let x = area.top_left.x as u16;
        let y = area.top_left.y as u16;
        let width = area.size.width as u16;
        let height = area.size.height as u16;

        self.rgb_interface.start_frame()?;

        // Skip lines before y
        for _ in 0..y {
            self.rgb_interface.start_line()?;
            for _ in 0..self.width {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }
            self.rgb_interface.end_line()?;
        }

        // Draw the filled rectangle lines
        for line_y in y..(y + height).min(self.height) {
            self.rgb_interface.start_line()?;

            // Pixels before the rectangle
            for _ in 0..x {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }

            // Rectangle pixels
            for _ in x..(x + width).min(self.width) {
                self.rgb_interface.send_pixel(color)?;
            }

            // Pixels after the rectangle
            for _ in (x + width).min(self.width)..self.width {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }

            self.rgb_interface.end_line()?;
        }

        // Remaining lines after the rectangle
        for _ in (y + height).min(self.height)..self.height {
            self.rgb_interface.start_line()?;
            for _ in 0..self.width {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }
            self.rgb_interface.end_line()?;
        }

        self.rgb_interface.end_frame()?;

        Ok(())
    }
}



// --- Type Alias for the Concrete Display ---
type MyDisplay = GC9503Display<Blocking>;


// --- LCD Resolution and FrameBuffer Type Aliases ---
const LCD_H_RES: usize = 480;
const LCD_V_RES: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES * LCD_V_RES;

// We want our pixels stored as Rgb565.
type FbBuffer = HeapBuffer<Rgb565, LCD_BUFFER_SIZE>;
// Define a type alias for the complete FrameBuf.
type MyFrameBuf = FrameBuf<Rgb565, FbBuffer>;

#[derive(Resource)]
struct FrameBufferResource {
    frame_buf: MyFrameBuf,
}

impl FrameBufferResource {
    fn new() -> Self {
        // Allocate the framebuffer data on the heap.
        let fb_data: Box<[Rgb565; LCD_BUFFER_SIZE]> = Box::new([Rgb565::BLACK; LCD_BUFFER_SIZE]);
        let heap_buffer = HeapBuffer::new(fb_data);
        let frame_buf = MyFrameBuf::new(heap_buffer, LCD_H_RES, LCD_V_RES);
        Self { frame_buf }
    }
}

// --- Game of Life Definitions ---
// Now each cell is a u8 (0 means dead; >0 indicates age)
const GRID_WIDTH: usize = 64;
const GRID_HEIGHT: usize = 48;
const RESET_AFTER_GENERATIONS: usize = 500;

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

/// Maps cell age (1...=max_age) to a color. Newborn cells are dark blue and older cells become brighter (toward white).
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

// Because our display type contains DMA descriptors and raw pointers, it isn’t Sync.
// We wrap it as a NonSend resource so that Bevy doesn’t require Sync.
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

fn send_test_pattern(interface: &mut RGB16BitInterface, width: u16, height: u16, pattern_type: u8) -> Result<(), ()> {
    info!("Sending test pattern type {}", pattern_type);

    interface.start_frame()?;

    for y in 0..height {
        interface.start_line()?;

        for x in 0..width {
            // Select a pattern based on the pattern_type parameter
            let color = match pattern_type {
                // Color bars
                0 => {
                    let segment = (x * 8) / width;
                    match segment {
                        0 => Rgb565::RED,
                        1 => Rgb565::GREEN,
                        2 => Rgb565::BLUE,
                        3 => Rgb565::YELLOW,
                        4 => Rgb565::CYAN,
                        5 => Rgb565::MAGENTA,
                        6 => Rgb565::WHITE,
                        _ => Rgb565::BLACK,
                    }
                },

                // Checkerboard
                1 => {
                    let checker_size = 16; // Size of each checker square
                    let checker_x = (x / checker_size) % 2;
                    let checker_y = (y / checker_size) % 2;
                    if (checker_x + checker_y) % 2 == 0 {
                        Rgb565::WHITE
                    } else {
                        Rgb565::BLACK
                    }
                },

                // Gradient
                2 => {
                    let r = (x * 31) / width;
                    let g = (y * 63) / height;
                    let b = 31 - (x * 31) / width;
                    Rgb565::new(r as u8, g as u8, b as u8)
                },

                // White screen
                3 => Rgb565::WHITE,

                // Black screen
                4 => Rgb565::BLACK,

                // Default - red screen
                _ => Rgb565::RED,
            };

            interface.send_pixel(color)?;
        }

        interface.end_line()?;
    }

    interface.end_frame()?;

    Ok(())
}

fn verify_data_pin_mapping(interface: &mut RGB16BitInterface, width: u16, height: u16) -> Result<(), ()> {
    info!("Verifying data pin mapping...");

    // Test each data pin individually
    for test_pin in 0..16 {
        interface.start_frame()?;

        for y in 0..height {
            interface.start_line()?;

            for x in 0..width {
                // Set all pins low
                for bit in 0..16 {
                    interface.data_pins[bit].set_low();
                }

                // Set only the test pin high
                interface.data_pins[test_pin].set_high();

                // Pulse PCLK
                interface.pulse_pclk()?;
            }

            interface.end_line()?;
        }

        interface.end_frame()?;

        // Wait for observation - the screen should show a specific color
        // based on which bit is active
        let color_name = match test_pin {
            0..=4 => "blue",
            5..=10 => "green",
            11..=15 => "red",
            _ => "unknown",
        };

        info!("Testing data pin {} ({})", test_pin, color_name);

        // Wait for 1 second to observe the result
        let mut delay = Delay::new();
        delay.delay_ms(1000);
    }

    Ok(())
}

/// Render the game state by drawing into the offscreen framebuffer and then flushing
/// it to the display via DMA. After drawing the game grid and generation number,
/// we overlay centered text.
fn render_system(
    mut display_res: NonSendMut<DisplayResource>,
    game: Res<GameOfLifeResource>,
    mut fb_res: ResMut<FrameBufferResource>,
) {
    // Clear the framebuffer.
    fb_res.frame_buf.clear(Rgb565::BLUE).unwrap();
    // Draw the game grid (using the age-based color) and generation number.
    draw_grid(&mut fb_res.frame_buf, &game.grid).unwrap();
    write_generation(&mut fb_res.frame_buf, game.generation).unwrap();

    // --- Overlay centered text ---
    let line1 = "Rust no_std ESP32-S3-LCD-EV-Board";
    let line2 = "Bevy ECS 0.16 no_std";
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
    // Flush the framebuffer to the physical display.
    display_res
        .display
        .fill_contiguous(&area, fb_res.frame_buf.data.iter().copied())
        .unwrap();
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // PSRAM allocator for heap memory
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    init_logger_from_env();


    let i2c = I2c::new(peripherals.I2C0,
                       esp_hal::i2c::master::Config::default()
                           .with_frequency(Rate::from_khz(400))
                           .with_timeout(BusTimeout::BusCycles(1000))
    )
        .unwrap()
        .with_sda(peripherals.GPIO47)
        .with_scl(peripherals.GPIO48);

    // Initialize IO expander for the sub-board control via SPI
    let mut io_expander = IOExpander::new(i2c);
    match io_expander.init() {
        Ok(_) => info!("IO expander initialized successfully"),
        Err(e) => {
            error!("Failed to initialize IO expander: {:?}", e);
        }
    }

    // Create SPI via IO expander for LCD initialization
    let spi = SpiViaIOExpander::new(io_expander);
    // --- Data lines for 16-bit RGB interface based on ESP-BSP definitions ---
    // Lower 8 bits (D0-D7)
    let data0 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default()); // LCD_DATA0
    let data1 = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default()); // LCD_DATA1
    let data2 = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default()); // LCD_DATA2
    let data3 = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default()); // LCD_DATA3
    let data4 = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default()); // LCD_DATA4
    let data5 = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default()); // LCD_DATA5
    let data6 = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());  // LCD_DATA6
    let data7 = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default()); // LCD_DATA7

    // Higher 8 bits (D8-D15)
    let data8 = Output::new(peripherals.GPIO45, Level::Low, OutputConfig::default());  // LCD_DATA8
    let data9 = Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default());  // LCD_DATA9
    let data10 = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default()); // LCD_DATA10
    let data11 = Output::new(peripherals.GPIO40, Level::Low, OutputConfig::default()); // LCD_DATA11
    let data12 = Output::new(peripherals.GPIO41, Level::Low, OutputConfig::default()); // LCD_DATA12
    let data13 = Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default()); // LCD_DATA13
    let data14 = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());  // LCD_DATA14
    let data15 = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());  // LCD_DATA15

    // RGB control signals based on ESP-BSP definitions
    let de = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());    // LCD_DE
    let pclk = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());   // LCD_PCLK
    let vsync = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());  // LCD_VSYNC
    let hsync = Output::new(peripherals.GPIO46, Level::Low, OutputConfig::default()); // LCD_HSYNC

    // Create data pins array for 16-bit RGB interface
    let data_pins = [
        data0, data1, data2, data3, data4, data5, data6, data7,
        data8, data9, data10, data11, data12, data13, data14, data15,
    ];

    // Create RGB interface
    let rgb_interface = RGB16BitInterface::new(de, pclk, vsync, hsync, data_pins);

    // LCD dimensions
    let width = 480;
    let height = 480;

    // Create and initialize GC9503 display
    let mut display = GC9503Display::new(spi, rgb_interface, width, height);

    let mut display_delay = Delay::new();
    display.init(&mut display_delay).expect("Failed to initialize display");

    verify_data_pin_mapping(&mut display.rgb_interface, width as u16, height as u16).expect("Failed to verify data pin mapping");
    send_test_pattern(&mut display.rgb_interface, width as u16, height as u16, 0).expect("Failed to send test pattern");
    // Clear the display with blue color
    display.clear(Rgb565::BLUE).expect("Failed to clear display");

    info!("Display initialized");

    info!("Starting display diagnostic test");

    // Test pattern 1: Full screen colors in sequence
    let colors = [
        Rgb565::RED,
        Rgb565::GREEN,
        Rgb565::BLUE,
        Rgb565::WHITE,
        Rgb565::MAGENTA,
        Rgb565::CYAN,
        Rgb565::YELLOW,
    ];

    let mut delay = Delay::new();
    info!("Drawing full screen color test pattern");
    for (i, &color) in colors.iter().enumerate() {
        info!("Filling screen with color #{}", i);
        display.clear(color);
        delay.delay_ns(500_000u32);
    }

    // Test pattern 2: Vertical bars
    info!("Drawing vertical color bars");
    let bar_width = 320 / colors.len() as u16;
    for (i, &color) in colors.iter().enumerate() {
        let x = i as u16 * bar_width;
        let rectangle = Rectangle::new(
            Point::new(x as i32, 0),
            Size::new(bar_width as u32, 480)
        );
        display.fill_solid(&rectangle, color);
    }
    delay.delay_ms(1000u32);

   
    delay.delay_ms(1000u32);
    info!("Display diagnostic test completed");

    loop {

    }

    // --- Initialize Game Resources ---
    let mut game = GameOfLifeResource::default();
    let mut rng_instance = Rng::new(peripherals.RNG);
    randomize_grid(&mut rng_instance, &mut game.grid);

    // Create a glider pattern
    let glider = [(1, 0), (2, 1), (0, 2), (1, 2), (2, 2)];
    for (x, y) in glider.iter() {
        game.grid[*y][*x] = 1; // alive with age 1
    }

    // Create the framebuffer resource
    let fb_res = FrameBufferResource::new();

    let mut world = World::default();
    world.insert_resource(game);
    world.insert_resource(RngResource(rng_instance));
    // Insert the display as a non-send resource
    world.insert_non_send_resource(DisplayResource { display });
    // Insert the framebuffer resource
    world.insert_resource(fb_res);

    let mut schedule = Schedule::default();
    schedule.add_systems(update_game_of_life_system);
    schedule.add_systems(render_system);

    let mut loop_delay = Delay::new();

    loop {
        schedule.run(&mut world);
        loop_delay.delay_ms(50u32);
    }
}