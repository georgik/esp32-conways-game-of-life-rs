#![no_std]
#![no_main]

extern crate alloc;

use alloc::{boxed::Box, vec};

use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{
        DmaDescriptor, DmaTxBuf, ExternalBurstConfig,
        aligned::{DmaAlignedMut, InternalMemory},
    },
    gpio::{Level, Output, OutputConfig},
    lcd_cam::{
        LcdCam,
        lcd::{
            ClockMode, Phase, Polarity,
            dpi::{Config, Dpi, Format, FrameTiming},
        },
    },
    main,
    psram::Psram,
    rng::Rng,
    time::Rate,
};

use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("PANIC: {}", info);

    loop {
        core::hint::spin_loop();
    }
}

// -----------------------------------------------------------------------------
// Display
// -----------------------------------------------------------------------------

const LCD_WIDTH: usize = 800;
const LCD_HEIGHT: usize = 480; // 480
const FRAME_BYTES: usize = LCD_WIDTH * LCD_HEIGHT * 2;

const HSYNC_WIDTH: usize = 4;
const H_BACK_PORCH: usize = 8;
const H_FRONT_PORCH: usize = 8;

const VSYNC_WIDTH: usize = 4;
const V_BACK_PORCH: usize = 8;
const V_FRONT_PORCH: usize = 8;

const DMA_ALIGNMENT: usize = 64;

// Conservative maximum descriptor chunk for 64-byte alignment.
const DESCRIPTOR_CHUNK: usize = 4096 - DMA_ALIGNMENT;
const DESCRIPTOR_COUNT: usize = FRAME_BYTES.div_ceil(DESCRIPTOR_CHUNK);

// One ordinary, linear DMA descriptor array in internal memory.
#[unsafe(link_section = ".dma")]
static mut TX_DESCRIPTORS:
    InternalMemory<[DmaDescriptor; DESCRIPTOR_COUNT]> =
    InternalMemory::new([
        DmaDescriptor::EMPTY;
        DESCRIPTOR_COUNT
    ]);

// At approximately 61 Hz, three frames is about 49 ms.
// Rendering adds some time between transfers.
const FRAMES_PER_GENERATION: usize = 5;

// -----------------------------------------------------------------------------
// Allocate a 64-byte-aligned framebuffer from the PSRAM heap.
// -----------------------------------------------------------------------------

fn allocate_framebuffer() -> DmaAlignedMut<'static, [u8]> {
    // Extra bytes let us align the start of the framebuffer.
    // The allocation remains valid for the application's lifetime.
    let allocation: &'static mut [u8] =
        Box::leak(
            vec![0u8; FRAME_BYTES + DMA_ALIGNMENT - 1]
                .into_boxed_slice(),
        );

    let offset = allocation
        .as_ptr()
        .align_offset(DMA_ALIGNMENT);

    assert!(offset < DMA_ALIGNMENT);

    let pixels =
        &mut allocation[offset..offset + FRAME_BYTES];

    assert_eq!(pixels.as_ptr() as usize % DMA_ALIGNMENT, 0);
    assert_eq!(pixels.len() % DMA_ALIGNMENT, 0);

    DmaAlignedMut::new(pixels)
        .expect("Framebuffer is not DMA-aligned")
}

// -----------------------------------------------------------------------------
// Game of Life
// -----------------------------------------------------------------------------

const CELL_SIZE: usize = 8;
const GRID_WIDTH: usize = LCD_WIDTH / CELL_SIZE;
const GRID_HEIGHT: usize = LCD_HEIGHT / CELL_SIZE;
const GRID_CELLS: usize = GRID_WIDTH * GRID_HEIGHT;

fn randomize_grid(rng: &mut Rng, grid: &mut [u8]) {
    rng.read(grid);

    for cell in grid.iter_mut() {
        *cell &= 1;
    }
}

fn update_game_of_life(current: &[u8], next: &mut [u8]) {
    for y in 0..GRID_HEIGHT {
        let above = (y + GRID_HEIGHT - 1) % GRID_HEIGHT;
        let below = (y + 1) % GRID_HEIGHT;

        for x in 0..GRID_WIDTH {
            let left = (x + GRID_WIDTH - 1) % GRID_WIDTH;
            let right = (x + 1) % GRID_WIDTH;

            let neighbors = [
                above * GRID_WIDTH + left,
                above * GRID_WIDTH + x,
                above * GRID_WIDTH + right,
                y * GRID_WIDTH + left,
                y * GRID_WIDTH + right,
                below * GRID_WIDTH + left,
                below * GRID_WIDTH + x,
                below * GRID_WIDTH + right,
            ];

            let alive_neighbors = neighbors
                .iter()
                .filter(|&&index| current[index] != 0)
                .count();

            let index = y * GRID_WIDTH + x;
            let age = current[index];

            next[index] = if age != 0 {
                if alive_neighbors == 2 || alive_neighbors == 3 {
                    age.saturating_add(1)
                } else {
                    0
                }
            } else if alive_neighbors == 3 {
                1
            } else {
                0
            };
        }
    }
}

// -----------------------------------------------------------------------------
// RGB565 rendering
// -----------------------------------------------------------------------------

fn rgb565(r: u16, g: u16, b: u16) -> u16 {
    (r << 11) | (g << 5) | b
}

fn age_to_color(age: u8) -> u16 {
    if age == 0 {
        return 0;
    }

    let age = u16::from(age.min(10));
    let r = (31 * age + 5) / 10;
    let g = (63 * age + 5) / 10;

    rgb565(r, g, 31)
}

fn render_grid_pixels(pixels: &mut [u8], grid: &[u8]) {
    assert!(CELL_SIZE >= 2);//2
    assert_eq!(grid.len(), GRID_CELLS);
    assert_eq!(GRID_WIDTH * CELL_SIZE, LCD_WIDTH);
    assert_eq!(GRID_HEIGHT * CELL_SIZE, LCD_HEIGHT);
    assert_eq!(pixels.len(), FRAME_BYTES);

    const SCANLINE_BYTES: usize = LCD_WIDTH * 2;
    const CELL_ROW_BYTES: usize = CELL_SIZE * 2;

    let border_color = rgb565(28, 56, 28).to_le_bytes();

    for cell_y in 0..GRID_HEIGHT {
        let top_start = cell_y * CELL_SIZE * SCANLINE_BYTES;
        let inner_start = top_start + SCANLINE_BYTES;

        for cell_x in 0..GRID_WIDTH {
            let age = grid[cell_y * GRID_WIDTH + cell_x];

            let cell_offset = cell_x * CELL_ROW_BYTES;
            let top_cell = top_start + cell_offset;
            let inner_cell = inner_start + cell_offset;

            if age == 0 {
                pixels[top_cell..top_cell + CELL_ROW_BYTES].fill(0);
                pixels[inner_cell..inner_cell + CELL_ROW_BYTES].fill(0);
            } else {
                let fill_color = age_to_color(age).to_le_bytes();

                for pixel in pixels
                    [top_cell..top_cell + CELL_ROW_BYTES]
                    .chunks_exact_mut(2)
                {
                    pixel.copy_from_slice(&border_color);
                }

                for pixel in pixels
                    [inner_cell..inner_cell + CELL_ROW_BYTES]
                    .chunks_exact_mut(2)
                {
                    pixel.copy_from_slice(&fill_color);
                }

                // Left and right borders.
                pixels[inner_cell..inner_cell + 2]
                    .copy_from_slice(&border_color);

                let right = inner_cell + CELL_ROW_BYTES - 2;
                pixels[right..right + 2]
                    .copy_from_slice(&border_color);
            }
        }

        // Copy the interior scanline into the remaining interior rows.
        for local_y in 2..CELL_SIZE - 1 {
            let destination = top_start + local_y * SCANLINE_BYTES;

            pixels.copy_within(
                inner_start..inner_start + SCANLINE_BYTES,
                destination,
            );
        }

        // Copy the top border to the bottom border.
        let bottom_start =
            top_start + (CELL_SIZE - 1) * SCANLINE_BYTES;

        pixels.copy_within(
            top_start..top_start + SCANLINE_BYTES,
            bottom_start,
        );
    }
}

fn render_test_bands(pixels: &mut [u8]) {
    assert_eq!(pixels.len(), FRAME_BYTES);

    let colors = [
        rgb565(0, 63, 0),  // Second quarter: green
        rgb565(31, 0, 0),  // Top quarter: red
        rgb565(0, 0, 31),  // Third quarter: blue
        rgb565(31, 63, 0), // Bottom quarter: yellow
        rgb565(31, 63, 31),  // Third quarter: white
        rgb565(31, 31, 31), // Bottom quarter: yellow
    ];

    for (y, row) in pixels
        .chunks_exact_mut(LCD_WIDTH * 2)
        .enumerate()
    {
        let color = colors[y * 6 / LCD_HEIGHT].to_le_bytes();

        for pixel in row.chunks_exact_mut(2) {
            pixel.copy_from_slice(&color);
        }
    }
}

fn render_test_bands2(pixels: &mut [u8]) {
    assert_eq!(pixels.len(), FRAME_BYTES);

    let colors = [
        rgb565(31, 63, 31),  // Third quarter: white
        rgb565(31, 31, 31), // Bottom quarter: yellow
        rgb565(31, 63, 0), // Bottom quarter: yellow
        rgb565(0, 0, 31),  // Third quarter: blue
        rgb565(0, 63, 0),  // Second quarter: green
        rgb565(31, 0, 0),  // Top quarter: red
    ];

    for (y, row) in pixels
        .chunks_exact_mut(LCD_WIDTH * 2)
        .enumerate()
    {
        let color = colors[y * 6 / LCD_HEIGHT].to_le_bytes();

        for pixel in row.chunks_exact_mut(2) {
            pixel.copy_from_slice(&color);
        }
    }
}



// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

#[main]
fn main() -> ! {
    let peripherals =
        esp_hal::init(
            esp_hal::Config::default()
                .with_cpu_clock(CpuClock::_240MHz),
        );

    println!("1: HAL initialized");

    // Initialize the PSRAM heap before allocating the framebuffer.
    let psram = Psram::new(peripherals.PSRAM, Default::default());
    esp_alloc::psram_allocator!(&psram);

    println!("2: PSRAM allocator initialized");

    let delay = Delay::new();

    let mut lcd_disp = Output::new(
        peripherals.GPIO38,
        Level::Low,
        OutputConfig::default(),
    );

    // Allow the display hardware to settle.
    delay.delay_millis(200);

    // Allocate one framebuffer in PSRAM.
    let pixels = allocate_framebuffer();

    println!("Framebuffer address: {:#010X}",
        pixels.as_ptr() as usize);
    println!("Framebuffer bytes: {}", pixels.len());

    // Retrieve the single descriptor array.
    let descriptors = unsafe {
        (&mut *(&raw mut TX_DESCRIPTORS))
            .get_mut()
            .unsize()
    };

    // Use the HAL's ordinary linear DMA descriptor chain.
    let mut dma_tx = DmaTxBuf::new_with_config(
        descriptors,
        pixels,
        ExternalBurstConfig::Size64,
    )
    .expect("Failed to create DMA transmit buffer");

    dma_tx.set_length(FRAME_BYTES);

    println!("3: One PSRAM framebuffer and linear DMA buffer ready");

    // Initialize the Game of Life grids.
    let mut current =
        vec![0u8; GRID_CELLS].into_boxed_slice();

    let mut next =
        vec![0u8; GRID_CELLS].into_boxed_slice();

    let mut rng = Rng::new();
    randomize_grid(&mut rng, &mut current);

    // Software framebuffer: only the CPU renders into this allocation.
    // It is separate from the aligned framebuffer owned by dma_tx.
    let mut render_buffer = vec![0u8; FRAME_BYTES].into_boxed_slice();

    // Render a complete image first.
    render_grid_pixels(&mut render_buffer, &current);

    // Then copy the finished image into the DMA framebuffer.
    // The data is already in little-endian RGB565 format.
    dma_tx
        .as_mut_slice()
        .copy_from_slice(&render_buffer);

    println!("3b: Separate rendering and DMA framebuffers initialized");

    // -------------------------------------------------------------------------
    // Configure the display. GPIOs and timing remain unchanged.
    // -------------------------------------------------------------------------

    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let config = Config::default()
        .with_frequency(Rate::from_mhz(25))
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftHigh,
        })
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: LCD_WIDTH,
            horizontal_total_width:
                HSYNC_WIDTH
                    + H_BACK_PORCH
                    + LCD_WIDTH
                    + H_FRONT_PORCH,

            horizontal_blank_front_porch:
                HSYNC_WIDTH + H_BACK_PORCH,

            vertical_active_height: LCD_HEIGHT,
            vertical_total_height:
                VSYNC_WIDTH
                    + V_BACK_PORCH
                    + LCD_HEIGHT
                    + V_FRONT_PORCH,

            vertical_blank_front_porch:
                VSYNC_WIDTH + V_BACK_PORCH,

            hsync_width: HSYNC_WIDTH,
            vsync_width: VSYNC_WIDTH,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let mut dpi = Dpi::new(
        lcd_cam.lcd,
        peripherals.DMA_AXI_CH2,
        config,
    )
    .expect("Failed to configure RGB/DPI")
    .with_vsync(peripherals.GPIO45)
    .with_hsync(peripherals.GPIO44)
    .with_de(peripherals.GPIO43)
    .with_pclk(peripherals.GPIO40)
    // Blue
    .with_data0(peripherals.GPIO8)
    .with_data1(peripherals.GPIO9)
    .with_data2(peripherals.GPIO10)
    .with_data3(peripherals.GPIO11)
    .with_data4(peripherals.GPIO12)
    // Green
    .with_data5(peripherals.GPIO13)
    .with_data6(peripherals.GPIO14)
    .with_data7(peripherals.GPIO15)
    .with_data8(peripherals.GPIO16)
    .with_data9(peripherals.GPIO17)
    .with_data10(peripherals.GPIO18)
    // Red
    .with_data11(peripherals.GPIO19)
    .with_data12(peripherals.GPIO33)
    .with_data13(peripherals.GPIO34)
    .with_data14(peripherals.GPIO35)
    .with_data15(peripherals.GPIO36);

    println!("4: Display configured");

    let mut display_enabled = false;
    let mut frames_since_generation = 0usize;

    println!("Starting staged-rendering frame loop");

    loop {
        // The DMA framebuffer already contains a complete image.
        //
        // Ownership moves into the transfer, so neither framebuffer
        // copying nor direct DMA-buffer modification happens here.
        let transfer = match dpi.send(false, dma_tx) {
            Ok(transfer) => {
                if !display_enabled {
                    lcd_disp.set_high();
                    display_enabled = true;
                }

                transfer
            }

            Err((error, returned_dpi, returned_dma_tx)) => {
                dpi = returned_dpi;
                dma_tx = returned_dma_tx;

                println!("LCD DMA send error: {:?}", error);
                delay.delay_millis(50);
                continue;
            }
        };

        // Recover the peripheral and DMA framebuffer after the
        // one-shot transfer finishes.
        let (result, returned_dpi, returned_dma_tx) =
            transfer.wait();

        dpi = returned_dpi;
        dma_tx = returned_dma_tx;

        if let Err(error) = result {
            println!("LCD DMA transfer error: {:?}", error);
            delay.delay_millis(50);
            continue;
        }

        frames_since_generation += 1;

        if frames_since_generation >= FRAMES_PER_GENERATION {
            // 1. Advance the simulation.
            update_game_of_life(&current, &mut next);
            core::mem::swap(&mut current, &mut next);

            // 2. Render into the SOFTWARE framebuffer.
            //
            // All cell drawing and scanline copy_within operations
            // now happen outside the DMA framebuffer.
            render_grid_pixels(
                &mut render_buffer,
                &current,
            );

            // 3. Copy the completed image into the DMA framebuffer.
            //
            // The previous transfer has finished. The next transfer
            // will not start until this entire copy is complete.
            dma_tx
                .as_mut_slice()
                .copy_from_slice(&render_buffer);

            frames_since_generation = 0;
        }

        // The next iteration transmits the completed DMA framebuffer.
        // On iterations without a generation update, it retransmits
        // the existing image without rewriting it.
    }
}