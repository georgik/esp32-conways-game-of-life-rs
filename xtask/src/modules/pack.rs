/// Package the minimal espbrew flash artifact for each project.
///
/// For a Rust no_std ESP release build we do **not** pre-assemble a single flash
/// image (the old `espflash save-image` approach). Instead we transfer just the
/// project markers plus the built ELF:
///
///   <out>/Cargo.toml
///   <out>/.cargo/config.toml
///   <out>/target/<triple>/release/<elf>
///
/// espbrew's autodetect (`RustESPDetector`) recognises these markers on the
/// flashing machine, reads the target triple from `.cargo/config.toml`, locates
/// the ELF at `target/<triple>/release`, and the cluster leader converts that
/// ELF into the bootloader / partitions / app images on the server. So the
/// `<triple>/release` layout must be preserved exactly.
use crate::modules::project::ProjectInfo;
use anyhow::{Context, Result};
use std::fs;
use std::path::{Path, PathBuf};

/// Package every project that has a `Cargo.toml` and a successful release build.
///
/// Each artifact is written to a sibling directory `pack-<project>` of the
/// project (i.e. at the repo root, next to the project itself) so the workflow
/// can upload one artifact per board.
pub fn pack_all_projects(projects: &[ProjectInfo], verbose: bool) -> Result<()> {
    println!("\n[PACK] Packaging minimal espbrew flash artifacts");
    println!("{}", "=".repeat(60));

    let mut packed = 0usize;
    let mut skipped = 0usize;

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        let out_dir = match project.path.parent() {
            Some(parent) => parent.join(format!("pack-{}", project.name)),
            None => PathBuf::from(format!("pack-{}", project.name)),
        };

        match pack_project(project, &out_dir, verbose) {
            Ok(()) => {
                println!("OK:    {} -> {}", project.name, out_dir.display());
                packed += 1;
            }
            Err(e) => {
                println!("SKIP:  {} ({})", project.name, e);
                skipped += 1;
            }
        }
    }

    println!("\nPack Summary: {} packed, {} skipped", packed, skipped);
    if packed == 0 {
        anyhow::bail!("no projects packed (nothing to upload)");
    }
    Ok(())
}

fn pack_project(project: &ProjectInfo, out_dir: &Path, verbose: bool) -> Result<()> {
    let cargo_config = project.path.join(".cargo").join("config.toml");
    if !cargo_config.exists() {
        anyhow::bail!("no .cargo/config.toml");
    }

    let config = fs::read_to_string(&cargo_config).context("read .cargo/config.toml")?;
    let triple =
        extract_target_triple(&config).context("target triple not found in .cargo/config.toml")?;

    let release_dir = project.path.join("target").join(&triple).join("release");
    if !release_dir.exists() {
        anyhow::bail!(
            "build dir {} not found (run `cargo xtask build` first)",
            release_dir.display()
        );
    }

    let elf = find_elf(&release_dir)?;

    // Preserve the target/<triple>/release layout espbrew's FindBuildDir expects.
    let target_rel = Path::new("target").join(&triple).join("release");
    let out_elf = out_dir.join(&target_rel).join(elf.file_name().unwrap());

    // Start from a clean output dir so a stale ELF from a previous run is not kept.
    if out_dir.exists() {
        fs::remove_dir_all(out_dir).ok();
    }
    fs::create_dir_all(&out_elf.parent().context("elf parent")?)?;

    // Cargo.toml + .cargo/config.toml are the markers espbrew detects on.
    fs::copy(project.path.join("Cargo.toml"), out_dir.join("Cargo.toml"))
        .context("copy Cargo.toml")?;
    fs::create_dir_all(out_dir.join(".cargo"))?;
    fs::copy(&cargo_config, out_dir.join(".cargo").join("config.toml"))
        .context("copy .cargo/config.toml")?;
    // The built ELF — espbrew converts it to bootloader/partitions/app on the server.
    fs::copy(&elf, &out_elf).context("copy ELF")?;

    if verbose {
        println!("   triple:  {}", triple);
        println!(
            "   elf:     {} ({} bytes)",
            elf.display(),
            fs::metadata(&elf)?.len()
        );
        println!("   output:  {}", out_dir.display());
    }
    Ok(())
}

/// Extract the build target triple from `.cargo/config.toml`, mirroring
/// espbrew's `RustESPDetector::extractTargetTriple`: prefer a
/// `[target.<triple>]` header, then fall back to `target = "<triple>"` under
/// `[build]`.
fn extract_target_triple(config: &str) -> Option<String> {
    for line in config.lines() {
        let line = line.trim();

        if let Some(rest) = line.strip_prefix("[target.") {
            if let Some(end) = rest.find(']') {
                let triple = rest[..end].trim();
                if !triple.is_empty() {
                    return Some(triple.to_string());
                }
            }
        }

        if line.starts_with("target") && line.contains('=') {
            let parts: Vec<&str> = line.splitn(2, '=').collect();
            if parts.len() == 2 {
                let triple = parts[1].trim().trim_matches('"');
                if !triple.is_empty() && !triple.starts_with('$') {
                    return Some(triple.to_string());
                }
            }
        }
    }
    None
}

/// Find the ELF in the release dir, mirroring espbrew's
/// `RustESPDetector::GetArtifacts`: skip `.d`/`.o`/`.a`/`.rmeta`/`.rlib` and pick
/// the largest remaining file (the executable).
fn find_elf(release_dir: &Path) -> Result<PathBuf> {
    let mut best: Option<(u64, PathBuf)> = None;

    for entry in fs::read_dir(release_dir).context("read release dir")? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_str().unwrap_or_default();

        if name.ends_with(".d")
            || name.ends_with(".o")
            || name.ends_with(".a")
            || name.ends_with(".rmeta")
            || name.ends_with(".rlib")
        {
            continue;
        }

        let len = entry.metadata().map(|m| m.len()).unwrap_or(0);
        // espbrew treats anything > 10 KB (or executable) as the binary; the
        // largest file in the release dir is the ELF, so take the max.
        if len < 10_000 {
            continue;
        }
        if best.as_ref().map(|(l, _)| len > *l).unwrap_or(true) {
            best = Some((len, entry.path()));
        }
    }

    match best {
        Some((_, path)) => Ok(path),
        None => anyhow::bail!("no ELF found in {}", release_dir.display()),
    }
}
