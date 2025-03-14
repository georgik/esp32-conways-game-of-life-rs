# ESP32 Conway's Game of Life in Rust

Implementation of Conway's Game of Life Rust Bare Metal.

[![Wokwi](https://img.shields.io/endpoint?url=https%3A%2F%2Fwokwi.com%2Fbadge%2Fclick-to-simulate.json)](https://wokwi.com/projects/380370193649185793)

![ESP32 Conways Game of Life in Rust](esp32-conways-game-of-life-rs.png)

## Supported boards

### ESP32-S3-BOX-3

- https://github.com/espressif/esp-box

The implementation is based on Rust no\_std, using mipidsi crate.

```
cd esp32-s3-box-3
cargo run --release
```

### ESP32-C6-LCD-1.47 Waveshare

[Rust Bare Metal no_std](https://developer.espressif.com/blog/2025/02/rust-esp-hal-beta/) with [Bevy ECS no_std](https://github.com/bevyengine/bevy/issues/15460) on 1.47 inch [ESP32-C6 LCD Waheshare](https://www.waveshare.com/esp32-c6-lcd-1.47.htm) with DMA and framebuffer - [Conway's Game of Life](https://github.com/georgik/esp32-conways-game-of-life-rs/tree/main/esp32-c6-waveshare-1_47):

<video src="https://github.com/user-attachments/assets/e9d48ff7-b14c-4874-9521-fe59e915bc76" controls width="640">
View the video [here](https://github.com/user-attachments/assets/e9d48ff7-b14c-4874-9521-fe59e915bc76).
</video>

The implementation is based on Rust no\_std and Bevy 0.15 no\_std, plus mipidsi crate

```
cd esp32-c6-waveshare-1_47
cargo run --release
```

### ESP32-C3-LCDKit

Limitation: Framebuffer fits only to 240x190 pixels. Might be cause by allocation on stack instead of heap.

```
cd esp32-c3-lcdkit
cargo run --release
```
