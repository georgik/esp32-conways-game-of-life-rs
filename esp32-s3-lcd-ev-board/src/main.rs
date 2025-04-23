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
use esp_println::{logger::init_logger_from_env, println};
use log::info;
use mipidsi::{Builder, models::GC9503CV};
use mipidsi::{
    options::ColorOrder,
};
use mipidsi::interface::{Generic8BitBus, ParallelInterface};
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


impl<Dm: esp_hal::DriverMode> IOExpander<Dm> {
    // TCA9554 IO Expander address
    const BSP_IO_EXPANDER_I2C_ADDRESS: u8 = 0x20;

    // Registers for TCA9554
    // These constants are correct according to the C code
    const REG_INPUT_PORT: u8 = 0x00;
    const REG_OUTPUT_PORT: u8 = 0x01;
    const REG_POLARITY_INVERSION: u8 = 0x02;
    const REG_CONFIG: u8 = 0x03;

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
        // Configure pins 1, 2, 3 as outputs (0 = output, 1 = input)
        let config = 0xFF & !(1 << Self::BSP_LCD_SUB_BOARD_2_SPI_CS |
            1 << Self::BSP_LCD_SUB_BOARD_2_SPI_SCK |
            1 << Self::BSP_LCD_SUB_BOARD_2_SPI_SDO);

        let mut write_buf = [Self::REG_CONFIG, config];
        self.i2c.write(self.address, &write_buf)?;

        // Set initial state of outputs (all high)
        let output_state = 0xFF;
        let mut write_buf = [Self::REG_OUTPUT_PORT, output_state];
        self.i2c.write(self.address, &write_buf)
    }

    fn set_pin(&mut self, pin: u8, level: bool) -> Result<(), I2cError> {
        // Read current output state
        let mut write_buf = [Self::REG_OUTPUT_PORT];
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
        let mut write_buf = [Self::REG_OUTPUT_PORT, output_state];
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

    // Send a pixel to the display
    fn send_pixel(&mut self, color: Rgb565) -> Result<(), ()> {
        // Extract 5-6-5 RGB components
        let raw_value = color.into_storage();

        // Set data pins according to the 16-bit RGB565 value
        for i in 0..16 {
            let bit_value = (raw_value >> i) & 0x01;
            if bit_value != 0 {
                self.data_pins[i].set_high();
            } else {
                self.data_pins[i].set_low();
            }
        }

        // Pulse PCLK to latch the data
        self.pclk.set_high();
        self.pclk.set_low();

        Ok(())
    }

    // Set up for a frame transfer
    fn start_frame(&mut self) -> Result<(), ()> {
        self.vsync.set_high();
        self.hsync.set_low();
        self.de.set_low();
        Ok(())
    }

    // Start a new line
    fn start_line(&mut self) -> Result<(), ()> {
        self.hsync.set_high();
        self.de.set_high();
        Ok(())
    }

    // End a line
    fn end_line(&mut self) -> Result<(), ()> {
        self.hsync.set_low();
        self.de.set_low();
        Ok(())
    }

    // End frame
    fn end_frame(&mut self) -> Result<(), ()> {
        self.vsync.set_low();
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
        // Initialize the SPI interface
        self.spi.init()?;

        // Initial delay
        delay.delay_ms(120);

        // Software reset
        self.spi.send_command(0x01)?;
        delay.delay_ms(120);

        // Sleep out
        self.spi.send_command(0x11)?;
        delay.delay_ms(120);

        // Set frame rate - matches ESP-BSP values
        self.spi.send_command(0xB1)?;
        self.spi.send_data_multiple(&[0x02, 0x35, 0x36])?;

        // Set panel driving mode
        self.spi.send_command(0xB4)?;
        self.spi.send_data(0x00)?;

        // Set display inversion
        self.spi.send_command(0xB7)?;
        self.spi.send_data(0x02)?;

        // Set power control
        self.spi.send_command(0xC0)?;
        self.spi.send_data_multiple(&[0x18, 0x18])?;
        self.spi.send_command(0xC1)?;
        self.spi.send_data(0x41)?;
        self.spi.send_command(0xC2)?;
        self.spi.send_data(0x22)?;

        // Vcom voltage
        self.spi.send_command(0xC5)?;
        self.spi.send_data(0x30)?;

        // Memory access control (MADCTL)
        self.spi.send_command(0x36)?;
        self.spi.send_data(0x08)?; // RGB color order

        // Pixel format
        self.spi.send_command(0x3A)?;
        self.spi.send_data(0x55)?; // 16-bit color

        // Gamma correction - from ESP-BSP implementation
        self.spi.send_command(0xE0)?;
        self.spi.send_data_multiple(&[0x1F, 0x25, 0x22, 0x0B, 0x06, 0x0A, 0x4E, 0xC6, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;
        self.spi.send_command(0xE1)?;
        self.spi.send_data_multiple(&[0x1F, 0x3F, 0x3F, 0x0F, 0x1F, 0x0F, 0x46, 0x49, 0x3B, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x00])?;

        // Set RGB interface
        self.spi.send_command(0xB0)?;
        self.spi.send_data(0x00)?; // RGB interface enable

        // Turn on display
        self.spi.send_command(0x29)?;
        delay.delay_ms(20);

        Ok(())
    }

    fn clear(&mut self, color: Rgb565) -> Result<(), ()> {
        self.rgb_interface.start_frame()?;

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
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }
            self.rgb_interface.end_line()?;
        }

        // Draw relevant lines
        for line_y in y..end_y {
            self.rgb_interface.start_line()?;

            // Skip pixels before x
            for _ in 0..x {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }

            // Draw actual pixels from buffer
            let buffer_offset = ((line_y - y) * width) as usize;
            for offset_x in 0..actual_width {
                let buffer_idx = buffer_offset + offset_x as usize;
                if buffer_idx < buffer.len() {
                    self.rgb_interface.send_pixel(buffer[buffer_idx])?;
                } else {
                    self.rgb_interface.send_pixel(Rgb565::BLACK)?;
                }
            }

            // Fill remaining pixels in the line
            for _ in end_x..self.width {
                self.rgb_interface.send_pixel(Rgb565::BLACK)?;
            }

            self.rgb_interface.end_line()?;
        }

        // Fill the remaining lines
        for _ in end_y..self.height {
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

/// Render the game state by drawing into the offscreen framebuffer and then flushing
/// it to the display via DMA. After drawing the game grid and generation number,
/// we overlay centered text.
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


    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9);

    // Initialize IO expander for the sub-board control via SPI
    let mut io_expander = IOExpander::new(i2c);
    io_expander.init().expect("Failed to initialize IO expander");

    // Create SPI via IO expander for LCD initialization
    let spi = SpiViaIOExpander::new(io_expander);

    // --- Data lines for 16-bit RGB interface ---
    // Lower 8 bits (D0-D7)
    let data0 = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());  
    let data1 = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());  
    let data2 = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());  
    let data3 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());  
    let data4 = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());  
    let data5 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());  
    let data6 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());  
    let data7 = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());  

    // Higher 8 bits (D8-D15)
    let data8 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default()); 
    let data9 = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default()); 
    let data10 = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());
    let data11 = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let data12 = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let data13 = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let data14 = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let data15 = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());

    // RGB control signals
    let de = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());    
    let pclk = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());  
    let vsync = Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default()); 
    let hsync = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default()); 

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

    // Clear the display with blue color
    display.clear(Rgb565::BLUE).expect("Failed to clear display");

    info!("Display initialized");

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