//! Formal verification of the pure Game of Life core with Kani.
//!
//! The `esp32-c3-lcdkit` firmware depends on the sibling `game_of_life` crate
//! for its pure transition logic. That crate is host-verifiable, so we check it
//! with Kani (a bounded model checker: CBMC + a SAT solver). Kani symbolically
//! explores every execution path up to a bound and *proves* the proof-harness
//! `assert!`s can never fail -- catching logic bugs (e.g. an age that wraps
//! instead of saturates) with an exact counterexample.
//!
//! Two things run:
//!   1. `cargo test`           -- quick `core`-level sanity of the transition.
//!   2. `cargo kani --features kani` -- the actual formal proof of the laws:
//!        * memory safety  -- the toroidal neighbour index is always in bounds;
//!        * no wrap        -- a surviving cell's age is non-decreasing and 255
//!                            saturates to 255 (never wraps to 0);
//!        * thresholds     -- the 2/3 survival + 3-neighbour birth rules.
//!
//! Invocation (from the repo root):
//!     cargo xtask verify            # test + kani
//!     cargo xtask verify --kani-only
//!
//! This command targets a single crate, so it runs *before* ESP32 project
//! discovery in `main.rs` (like the gallery/wasm commands).

use anyhow::{Context, Result};
use std::path::PathBuf;
use tokio::process::Command as TokioCommand;

/// Location of the verifiable pure-logic crate, resolved at build time from the
/// xtask crate directory (so it is independent of the current working dir).
fn game_of_life_dir() -> PathBuf {
    let xtask_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    xtask_dir
        .parent()
        .expect("xtask manifest dir has no parent")
        .join("game_of_life")
}

/// Run a cargo command in `game_of_life`, inheriting the terminal so the user
/// sees the (possibly long) Kani output live. Returns the exit status.
async fn run_in_game_of_life(args: &[&str]) -> Result<std::process::ExitStatus> {
    let dir = game_of_life_dir();
    println!("\n$ cargo {}  (in {})", args.join(" "), dir.display());

    let status = TokioCommand::new("cargo")
        .args(args)
        .current_dir(&dir)
        .status()
        .await
        .with_context(|| format!("failed to spawn cargo {:?}", args))?;

    Ok(status)
}

/// Run the sanity test, then (unless `--kani-only`) the formal Kani proof.
pub async fn verify_formal_properties(kani_only: bool) -> Result<()> {
    println!("Formal verification of the Game of Life core (Kani)");
    println!("{}", "=".repeat(60));

    let dir = game_of_life_dir();
    if !dir.join("Cargo.toml").exists() {
        anyhow::bail!(
            "game_of_life crate not found at {} -- expected it to be a sibling of the xtask",
            dir.display()
        );
    }

    // 1. Quick sanity test (core-level, no SAT solver needed).
    if !kani_only {
        println!("\n[TEST] Running sanity tests with `cargo test`");
        let status = run_in_game_of_life(&["test"]).await?;
        if !status.success() {
            anyhow::bail!("sanity test failed (status {:?})", status);
        }
    } else {
        println!("\n[--kani-only] skipping `cargo test`");
    }

    // 2. The formal proof.
    println!("\n[PROOF] Running Kani with `cargo kani --features kani`");
    let status = run_in_game_of_life(&["kani", "--features", "kani"]).await?;

    if status.success() {
        println!("\n{}", "=".repeat(60));
        println!("SUCCESS: all formal properties verified for game_of_life");
        println!(
            "         (memory safety, age saturation, survival/birth thresholds)"
        );
    } else {
        anyhow::bail!("Kani verification failed (status {:?})", status);
    }

    Ok(())
}
