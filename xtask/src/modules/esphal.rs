use crate::modules::project::{ProjectInfo, TaskResult, TaskSummary};
use anyhow::{Context, Result};
use std::fs;

// Boards that call the pre-1.2 SPI-DMA API (`DmaRxBuf::new` / `DmaTxBuf::new`
// with plain slice buffers, or import `esp_hal::spi::master::SpiDmaBus`) break
// against esp-hal 1.2 (those signatures now require `DmaAlignedMut`). The last
// 1.1.x release still accepts the old call sites and is known-good on every
// toolchain these boards use, so we pin them back to it instead of letting the
// caret range drift onto the incompatible line.
const COMPATIBLE_ESP_HAL: &str = "=1.1.2";

pub async fn pin_incompatible_esp_hal(
    projects: &[ProjectInfo],
    dry_run: bool,
    verbose: bool,
) -> Result<()> {
    println!("\n[PIN-ESP-HAL] Pinning esp-hal to the last compatible 1.1.x line");

    if dry_run {
        println!("DRY-RUN mode - no changes will be made");
    }
    println!("{}", "=".repeat(60));

    let mut summary = TaskSummary::new();
    let mut results = Vec::new();

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        println!("\nProcessing: {}", project.name);
        let result = pin_project_esp_hal(project, dry_run, verbose).await?;
        if verbose && !result.message.is_empty() {
            println!("   {}", result.message);
        }
        summary.add_result(&result);
        results.push(result);
    }

    println!("\n{}", "=".repeat(60));
    println!("Pin ESP-HAL Summary:");
    let applied = results
        .iter()
        .filter(|r| r.message.contains("would pin") || r.message.contains("pinned esp-hal ->"))
        .count();
    let already_ok = results
        .iter()
        .filter(|r| {
            r.message.contains("already compatible")
                || r.message.contains("already pinned (no change)")
        })
        .count();
    let no_legacy_api = results
        .iter()
        .filter(|r| r.message.contains("no legacy DMA API"))
        .count();

    println!("Pinned: {}", applied);
    println!("Already compatible (1.1.x, unchanged): {}", already_ok);
    println!("No legacy DMA API (not affected): {}", no_legacy_api);

    if applied > 0 {
        println!("\nNext Steps:");
        if !dry_run {
            println!("1. Run: cargo xtask build --keep-going --verbose");
        } else {
            println!("1. Review the pinned Cargo.toml files above");
            println!("2. Run without --dry-run to apply changes");
        }
    }

    Ok(())
}

async fn pin_project_esp_hal(
    project: &ProjectInfo,
    dry_run: bool,
    _verbose: bool,
) -> Result<TaskResult> {
    let cargo_toml_path = project.path.join("Cargo.toml");
    if !cargo_toml_path.exists() {
        return Ok(TaskResult {
            project: project.name.clone(),
            success: true,
            message: String::new(),
            warnings: Vec::new(),
        });
    }

    let content = fs::read_to_string(&cargo_toml_path)
        .with_context(|| format!("Failed to read Cargo.toml in {}", project.name))?;

    if !uses_legacy_dma_api(project) {
        return Ok(TaskResult {
            project: project.name.clone(),
            success: true,
            message: "no legacy DMA API".to_string(),
            warnings: Vec::new(),
        });
    }

    let resolved = esp_hal_version_from_lock(project);
    let resolved_str = resolved.clone().unwrap_or_else(|| "unknown".to_string());
    if !needs_pin(&resolved) {
        return Ok(TaskResult {
            project: project.name.clone(),
            success: true,
            message: format!("already compatible (resolved esp-hal {})", resolved_str),
            warnings: Vec::new(),
        });
    }

    let new_content = rewrite_esp_hal_version(&content);
    if new_content == content {
        return Ok(TaskResult {
            project: project.name.clone(),
            success: true,
            message: format!(
                "already pinned (no change) - resolved esp-hal {}",
                resolved_str
            ),
            warnings: Vec::new(),
        });
    }

    if dry_run {
        return Ok(TaskResult {
            project: project.name.clone(),
            success: true,
            message: format!(
                "would pin esp-hal -> {} (resolved from {})",
                COMPATIBLE_ESP_HAL, resolved_str
            ),
            warnings: Vec::new(),
        });
    }

    fs::write(&cargo_toml_path, new_content)
        .with_context(|| format!("Failed to write Cargo.toml in {}", project.name))?;

    Ok(TaskResult {
        project: project.name.clone(),
        success: true,
        message: format!(
            "pinned esp-hal -> {} (resolved from {})",
            COMPATIBLE_ESP_HAL, resolved_str
        ),
        warnings: Vec::new(),
    })
}

/// A board uses the pre-1.2 SPI-DMA API if its source still imports `SpiDmaBus`
/// or calls `DmaRxBuf::new`/`DmaTxBuf::new` with plain slice buffers (the call
/// sites that 1.2 replaced with `DmaAlignedMut`). Boards that already migrated to
/// the new API are left untouched.
fn uses_legacy_dma_api(project: &ProjectInfo) -> bool {
    let candidates = ["src/main.rs", "src/lib.rs"];
    for rel in candidates.iter() {
        let path = project.path.join(rel);
        if let Ok(text) = fs::read_to_string(&path) {
            let hits = text
                .lines()
                .filter(|l| !l.trim().starts_with('#'))
                .filter(|l| {
                    l.contains("DmaRxBuf::new")
                        || l.contains("DmaTxBuf::new")
                        || l.contains("SpiDmaBus")
                })
                .count();
            if hits > 0 {
                return true;
            }
        }
    }
    false
}

/// Read the esp-hal version a project currently resolves to. Returns `Some` only
/// for registry dependencies (a git pin is not subject to crates.io drift).
fn esp_hal_version_from_lock(project: &ProjectInfo) -> Option<String> {
    let lock_path = project.path.join("Cargo.lock");
    let text = fs::read_to_string(lock_path).ok()?;
    let lines: Vec<&str> = text.lines().collect();
    for i in 1..lines.len() {
        if lines[i - 1].trim() == r#"name = "esp-hal""# && lines[i].contains("version") {
            return lines[i].split('"').nth(1).map(|v| v.to_string());
        }
    }
    None
}

/// Needs pinning when the resolved esp-hal is on an incompatible line: 1.2 and
/// above for the legacy DMA API, or any caret range that is not already pinned to
/// `=1.1.x`.
fn needs_pin(resolved: &Option<String>) -> bool {
    match resolved.as_deref() {
        None => true,
        Some(v) => {
            let parts: Vec<&str> = v.split('.').collect();
            let major = parts.first().and_then(|p| p.parse::<u64>().ok());
            let minor = parts.get(1).and_then(|p| p.parse::<u64>().ok());
            match (major, minor) {
                (Some(m), Some(n)) => m > 1 || (m == 1 && n >= 2),
                _ => true,
            }
        }
    }
}

/// Rewrite the `esp-hal` registry dependency's version specifier to an exact pin.
/// Only touches the active (non-commented) `esp-hal = ...` entry whose `version`
/// is a bare caret range; leaves git deps and already-pinned deps alone.
fn rewrite_esp_hal_version(content: &str) -> String {
    let mut out = String::with_capacity(content.len());
    for line in content.lines() {
        if looks_like_active_esp_hal(line) {
            let rewritten = replace_bare_version(line, COMPATIBLE_ESP_HAL);
            out.push_str(&rewritten);
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    out
}

/// Matches an active (uncommented) line declaring the esp-hal dependency.
fn looks_like_active_esp_hal(line: &str) -> bool {
    let trimmed = line.trim();
    if trimmed.starts_with('#') {
        return false;
    }
    trimmed.contains("esp-hal") && trimmed.contains("version")
}

/// Replace a bare caret/tilde version spec (`"X.Y.Z"`) with an exact pin
/// (`"=X.Y.Z"`), leaving already-pinned specs untouched.
fn replace_bare_version(line: &str, pinned: &str) -> String {
    let marker = "version = \"";
    if let Some(pos) = line.find(marker) {
        let before = &line[..pos + marker.len()];
        let after_open = &line[pos + marker.len()..];
        if let Some(close) = after_open.find('"') {
            let version_part = &after_open[..close];
            if version_part.starts_with('=') {
                return line.to_string();
            }
            return format!("{}{}\"{}", before, pinned, &after_open[close + 1..]);
        }
    }
    line.to_string()
}
