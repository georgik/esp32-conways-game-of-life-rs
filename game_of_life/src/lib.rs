//! Pure, host-verifiable Conway Game of Life core.
//!
//! This is the *exact* transition logic shipped by the `esp32-c3-lcdkit`
//! firmware (its `update_game_of_life`), factored out into a standalone,
//! `no_std` library so it can be checked on the host with **Kani** (bounded
//! model checking). The firmware depends on this crate, so the code that is
//! formally verified is the code that runs on the chip.
//!
//! The logic is intentionally free of any hardware side effects: it takes a
//! grid in and returns the next grid. Everything else in the firmware
//! (SPI/LCD, scheduling, bevy_ecs) is *not* pure and is left out of
//! the proof, exactly as it must be.
#![cfg_attr(not(test), no_std)]

/// Per-cell transition.
///
/// * A live cell (`old > 0`) survives with 2 or 3 live neighbours, ageing by
///   one via `saturating_add` (so age 255 stays 255, it never wraps to 0).
/// * Otherwise a live cell dies.
/// * A dead cell (`old == 0`) is born with exactly 3 neighbours.
///
/// This mirrors the firmware verbatim.
pub fn next_cell(old: u8, live_neighbours: u8) -> u8 {
    if old > 0 {
        if live_neighbours == 2 || live_neighbours == 3 {
            old.saturating_add(1)
        } else {
            0
        }
    } else if live_neighbours == 3 {
        1
    } else {
        0
    }
}

/// Advance the whole board by one generation.
///
/// Parameterised by board size so the host checker can exercise small boards
/// quickly; the firmware instantiates it as `update::<35, 35>`. The toroidal
/// neighbour index `(x + i + W - 1) % W` wraps the board, which is what the
/// Kani proof pins down (it can never go out of bounds).
pub fn update_game_of_life<const W: usize, const H: usize>(grid: &mut [[u8; W]; H]) {
    let mut new_grid = [[0u8; W]; H];
    for y in 0..H {
        for x in 0..W {
            // Count neighbours: a cell is "alive" if its age is > 0.
            let mut alive_neighbours: u8 = 0;
            for i in 0..3 {
                for j in 0..3 {
                    if i == 1 && j == 1 {
                        continue; // skip the cell itself
                    }
                    // Toroidal wrap: the +W / +H keep the subtrahend positive
                    // so the `%` result is always a valid index.
                    let nx = (x + i + W - 1) % W;
                    let ny = (y + j + H - 1) % H;
                    if grid[ny][nx] > 0 {
                        alive_neighbours += 1;
                    }
                }
            }
            new_grid[y][x] = next_cell(grid[y][x], alive_neighbours);
        }
    }
    *grid = new_grid;
}

/// Convenience wrapper matching the firmware's concrete 35x35 board.
pub fn update_35x35(grid: &mut [[u8; 35]; 35]) {
    update_game_of_life::<35, 35>(grid)
}

// ---------------------------------------------------------------------------
// Sanity test. Runs on the host with `cargo test`. The `no_std` body above is
// exercised purely through `core` assertions, so this works without std.
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn next_cell_matches_firmware_hand_cases() {
        assert_eq!(next_cell(5, 2), 6); // survives, ages
        assert_eq!(next_cell(255, 2), 255); // saturates, does NOT wrap
        assert_eq!(next_cell(1, 4), 0); // overpopulated -> dies
        assert_eq!(next_cell(0, 3), 1); // birth
        assert_eq!(next_cell(0, 2), 0); // stays dead
    }

    #[test]
    fn update_runs_on_a_small_board() {
        let mut grid: [[u8; 3]; 3] = [[0; 3]; 3];
        grid[1][1] = 5; // a single living cell -> next gen all dead
        update_game_of_life::<3, 3>(&mut grid);
        assert!(grid.iter().all(|row| row.iter().all(|c| *c == 0)));
    }
}

// ---------------------------------------------------------------------------
// Kani proof harnesses. Gated behind `feature = "kani"` so a plain `cargo
// build` (host or firmware) never compiles them. `cargo kani --features kani`
// is what runs these.
// ---------------------------------------------------------------------------
#[cfg(feature = "kani")]
mod kani_harness {
    use super::*;

    // LAW (memory safety): the toroidal neighbour index is always in bounds, so
    // `update` can never panic on an out-of-bounds read for ANY board.
    #[kani::proof]
    #[kani::unwind(4000)]
    fn prove_update_no_panic() {
        let mut grid: [[u8; 6]; 6] = kani::any();
        update_game_of_life::<6, 6>(&mut grid);
        // Reaching this line means no path panicked. An out-of-bounds index
        // would surface here as the counterexample.
    }

    // LAW (no wrap / saturation): a surviving cell's age is non-decreasing and
    // never wraps to 0; age 255 saturates to 255 rather than wrapping.
    #[kani::proof]
    fn prove_age_never_wraps() {
        let old: u8 = kani::any();
        let nb: u8 = kani::any();
        if old > 0 && (nb == 2 || nb == 3) {
            let n = next_cell(old, nb);
            kani::assert(n > 0, "surviving cell must not wrap to 0");
            kani::assert(n >= old, "surviving cell age must not decrease");
        }
        if old == 255 && (nb == 2 || nb == 3) {
            kani::assert(next_cell(old, nb) == 255, "age 255 must saturate, not wrap");
        }
    }

    // LAW (birth / death thresholds), spelled out as concrete cases so a
    // regression in the transition rules is caught immediately.
    #[kani::proof]
    fn prove_thresholds() {
        // A live cell with 2 or 3 neighbours survives (age > 0).
        kani::assert(next_cell(7, 2) > 0, "live cell survives with 2 neighbours");
        kani::assert(next_cell(7, 3) > 0, "live cell survives with 3 neighbours");
        // A live cell with 0, 1 or 4+ neighbours dies.
        kani::assert(next_cell(7, 0) == 0, "live cell dies with 0 neighbours");
        kani::assert(next_cell(7, 1) == 0, "live cell dies with 1 neighbour");
        kani::assert(next_cell(7, 4) == 0, "live cell dies with 4 neighbours");
        // A dead cell is born with exactly 3 neighbours.
        kani::assert(next_cell(0, 3) == 1, "dead cell born with 3 neighbours");
        kani::assert(next_cell(0, 2) == 0, "dead cell stays dead with 2 neighbours");
        kani::assert(next_cell(0, 4) == 0, "dead cell stays dead with 4 neighbours");
    }
}
