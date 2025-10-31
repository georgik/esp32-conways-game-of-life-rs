
use std::process::Command;
use std::path::Path;
use std::env;

fn main() {
    // Only do WASM build when building the native binary (not when building the lib for wasm)
    if env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default() != "wasm32" {
        println!("cargo:warning=Building WASM library...");

        // Step 1: Build the WASM library
        let wasm_build = Command::new("cargo")
            .args([
                "build",
                "--lib",
                "--target", "wasm32-unknown-unknown",
                "--release"
            ])
            .current_dir(env::var("CARGO_MANIFEST_DIR").unwrap())
            .status();

        match wasm_build {
            Ok(status) if status.success() => {
                println!("cargo:warning=WASM build successful");
            }
            Ok(status) => {
                panic!("WASM build failed with status: {}", status);
            }
            Err(e) => {
                panic!("Failed to run cargo build for WASM: {}", e);
            }
        }

        // Step 2: Run wasm-bindgen
        let wasm_file = "target/wasm32-unknown-unknown/release/wasm_conways_game_of_life.wasm";

        if Path::new(wasm_file).exists() {
            println!("cargo:warning=Running wasm-bindgen...");

            let bindgen = Command::new("wasm-bindgen")
                .args([
                    "--target", "web",
                    "--out-dir", "pkg",
                    wasm_file
                ])
                .current_dir(env::var("CARGO_MANIFEST_DIR").unwrap())
                .status();

            match bindgen {
                Ok(status) if status.success() => {
                    println!("cargo:warning=wasm-bindgen successful");
                }
                Ok(status) => {
                    panic!("wasm-bindgen failed with status: {}. Make sure it's installed: cargo install wasm-bindgen-cli", status);
                }
                Err(e) => {
                    panic!("Failed to run wasm-bindgen: {}. Make sure it's installed: cargo install wasm-bindgen-cli", e);
                }
            }
        } else {
            panic!("WASM file not found at {}", wasm_file);
        }

        println!("cargo:warning=WASM packaging complete! Starting server...");

        // Rerun build script if source files change
        println!("cargo:rerun-if-changed=src/lib.rs");
        println!("cargo:rerun-if-changed=src/bin/server.rs");
    }
}