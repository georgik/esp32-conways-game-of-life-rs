//! GitHub Pages screenshot gallery generator.
//!
//! Single source of truth for the hardware-snapshot gallery used by the
//! `flash-espbrew` workflows. It expands the per-board espbrew `snap` archives
//! (`<board>.zip`, as produced by `espbrew snap --zip`), copies the WASM
//! browser build into the site, and renders the gallery `index.html` from
//! `deploy/gallery/index.template.html` by replacing the `__BOARDS_CARDS__`
//! placeholder with one card per board.
//!
//! A card shows every image found in the board's archive (camera frame,
//! framebuffer PNG extracted from the serial log, or both). Boards listed via
//! `--expect-boards` but missing an archive get a placeholder card, so a
//! partial run still publishes a correct gallery.

use anyhow::{Context, Result};
use std::fs;
use std::path::{Path, PathBuf};

/// Instructions for one gallery build.
pub struct GalleryConfig {
    /// Directory containing per-board `<board>.zip` snap archives.
    pub zips_dir: PathBuf,
    /// Boards that receive a placeholder card when no archive exists.
    pub expected_boards: Vec<String>,
    /// WASM build directory copied to `<out>/wasm` (skipped when missing).
    pub wasm_dir: PathBuf,
    /// Gallery HTML template with the `__BOARDS_CARDS__` placeholder.
    pub template: PathBuf,
    /// Output site directory.
    pub out: PathBuf,
}

/// Expands the snap archives, copies the WASM build, and renders the gallery.
pub fn generate(config: &GalleryConfig) -> Result<()> {
    let screenshots_dir = config.out.join("screenshots");
    fs::create_dir_all(&screenshots_dir)
        .with_context(|| format!("create {}", screenshots_dir.display()))?;

    // Expand each board archive into its own subdir so frames from different
    // boards never mix. Boards are identified by the archive file stem.
    let mut boards: Vec<String> = Vec::new();
    if config.zips_dir.exists() {
        let mut entries: Vec<PathBuf> = fs::read_dir(&config.zips_dir)?
            .collect::<std::result::Result<Vec<_>, _>>()?
            .into_iter()
            .map(|e| e.path())
            .filter(|p| {
                p.extension()
                    .is_some_and(|ext| ext.eq_ignore_ascii_case("zip"))
            })
            .collect();
        entries.sort();
        for zip_path in &entries {
            let board = zip_path
                .file_stem()
                .and_then(|s| s.to_str())
                .context("archive name is not valid UTF-8")?
                .to_string();
            let board_dir = screenshots_dir.join(&board);
            fs::create_dir_all(&board_dir)?;
            extract_zip(zip_path, &board_dir).with_context(|| {
                format!(
                    "extract {} into {}",
                    zip_path.display(),
                    board_dir.display()
                )
            })?;
            boards.push(board);
        }
    }

    // Placeholder cards for boards that were expected but not snapshotted, so
    // a partial run still renders a complete gallery.
    for expected in &config.expected_boards {
        if !boards.contains(expected) {
            boards.push(expected.clone());
        }
    }
    // Deduplicate while keeping the order stable.
    boards.dedup();

    // Copy the WASM browser build into the site (kept live, never flashed).
    if config.wasm_dir.exists() {
        let wasm_out = config.out.join("wasm");
        if wasm_out.exists() {
            fs::remove_dir_all(&wasm_out)?;
        }
        copy_dir_recursive(&config.wasm_dir, &wasm_out)?;
    }

    // Render one card per board: every captured image, or a placeholder.
    let mut cards = String::new();
    for board in &boards {
        let board_dir = screenshots_dir.join(board);
        let images: Vec<String> = match fs::read_dir(&board_dir) {
            Ok(entries) => {
                let mut names: Vec<String> = entries
                    .flatten()
                    .map(|e| e.file_name().to_string_lossy().into_owned())
                    .filter(|name| {
                        let lower = name.to_ascii_lowercase();
                        lower.ends_with(".jpg")
                            || lower.ends_with(".jpeg")
                            || lower.ends_with(".png")
                    })
                    .collect();
                names.sort();
                names
            }
            Err(_) => Vec::new(),
        };

        cards.push_str("    <div class=\"card\">\n");
        cards.push_str(&format!("        <h2>Hardware snapshot ({board})</h2>\n"));
        if images.is_empty() {
            cards.push_str(
                "        <div class=\"placeholder\">No hardware snapshot captured</div>\n",
            );
        } else {
            for name in &images {
                cards.push_str(&format!(
                    "        <img class=\"screenshot\" src=\"screenshots/{board}/{name}\" alt=\"{board} hardware snapshot\">\n"
                ));
            }
        }
        cards.push_str("    </div>\n");
    }

    let template = fs::read_to_string(&config.template)
        .with_context(|| format!("read template {}", config.template.display()))?;
    let index = template.replace("__BOARDS_CARDS__", &cards);
    let index_path = config.out.join("index.html");
    fs::write(&index_path, index).with_context(|| format!("write {}", index_path.display()))?;

    println!("Gallery generated: {}", index_path.display());
    println!("Boards: {}", boards.join(", "));
    Ok(())
}

/// Extracts a zip archive into `dest`, refusing paths that escape it.
fn extract_zip(archive: &Path, dest: &Path) -> Result<()> {
    let file = fs::File::open(archive).with_context(|| format!("open {}", archive.display()))?;
    let mut zip =
        zip::ZipArchive::new(file).with_context(|| format!("read zip {}", archive.display()))?;

    for i in 0..zip.len() {
        let mut entry = zip.by_index(i)?;

        // In zip 9.x `name()` returns a Result because the stored entry name
        // may not be valid UTF-8. A name that can't be decoded can't be safely
        // used as a path, so decode it up front before extracting.
        let name = entry
            .name()
            .with_context(|| format!("zip {} has a non-UTF-8 entry name", archive.display()))?
            .into_owned();

        let Some(path) = entry.enclosed_name() else {
            anyhow::bail!("zip {} contains an unsafe path: {name}", archive.display());
        };
        let out_path = dest.join(&path);
        if entry.is_dir() {
            fs::create_dir_all(&out_path)?;
            continue;
        }
        if let Some(parent) = out_path.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut out_file = fs::File::create(&out_path)?;
        std::io::copy(&mut entry, &mut out_file)?;
    }
    Ok(())
}

/// Recursively copies a directory tree.
fn copy_dir_recursive(src: &Path, dest: &Path) -> Result<()> {
    fs::create_dir_all(dest)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let entry_type = entry.file_type()?;
        let target = dest.join(entry.file_name());
        if entry_type.is_dir() {
            copy_dir_recursive(&entry.path(), &target)?;
        } else {
            fs::copy(entry.path(), &target)?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_file(path: &Path, contents: &[u8]) {
        fs::create_dir_all(path.parent().unwrap()).unwrap();
        fs::write(path, contents).unwrap();
    }

    #[test]
    fn renders_cards_for_zips_and_placeholders() {
        let root = std::env::temp_dir().join(format!("xtask-gallery-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let zips_dir = root.join("zips");
        let wasm_dir = root.join("wasm");
        let out = root.join("site");

        // Board with a captured frame (as flat files inside a zip, like
        // espbrew snap --zip produces).
        let frame = root.join("frame.jpg");
        write_file(&frame, b"jpeg-bytes");
        let zip_path = zips_dir.join("esp32-s3-box-3.zip");
        write_file(&zip_path, b""); // placeholder, replaced below
        let zip_file = fs::File::create(&zip_path).unwrap();
        let mut zw = zip::ZipWriter::new(zip_file);
        zw.start_file("snap-1.jpg", zip::write::SimpleFileOptions::default())
            .unwrap();
        std::io::copy(&mut fs::File::open(&frame).unwrap(), &mut zw).unwrap();
        zw.finish().unwrap();

        // WASM build gets copied into the site.
        write_file(&wasm_dir.join("index.html"), b"<html>wasm</html>");

        // Template with the boards placeholder.
        let template = root.join("template.html");
        write_file(&template, b"<html><body>__BOARDS_CARDS__</body></html>");

        let config = GalleryConfig {
            zips_dir: zips_dir.clone(),
            expected_boards: vec!["esp32-s3-box-3".into(), "m5stack-atom-s3".into()],
            wasm_dir: wasm_dir.clone(),
            template: template.clone(),
            out: out.clone(),
        };
        generate(&config).unwrap();

        let index = fs::read_to_string(out.join("index.html")).unwrap();
        assert!(index.contains("<h2>Hardware snapshot (esp32-s3-box-3)</h2>"));
        assert!(index.contains("src=\"screenshots/esp32-s3-box-3/snap-1.jpg\""));
        assert!(index.contains("<h2>Hardware snapshot (m5stack-atom-s3)</h2>"));
        assert!(index.contains("No hardware snapshot captured"));
        assert!(!index.contains("__BOARDS_CARDS__"));
        assert_eq!(
            fs::read_to_string(out.join("wasm/index.html")).unwrap(),
            "<html>wasm</html>"
        );
        // The frame was expanded out of the archive.
        assert_eq!(
            fs::read(out.join("screenshots/esp32-s3-box-3/snap-1.jpg")).unwrap(),
            b"jpeg-bytes"
        );

        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn framebuffer_png_and_camera_jpg_both_render() {
        let root = std::env::temp_dir().join(format!("xtask-gallery-test2-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let zips_dir = root.join("zips");
        let out = root.join("site");
        let template = root.join("template.html");
        write_file(&template, b"__BOARDS_CARDS__");

        let zip_path = zips_dir.join("board.zip");
        write_file(&zip_path, b"");
        let zip_file = fs::File::create(&zip_path).unwrap();
        let mut zw = zip::ZipWriter::new(zip_file);
        for name in ["snap-1.jpg", "framebuffer-1.png"] {
            zw.start_file(name, zip::write::SimpleFileOptions::default())
                .unwrap();
            std::io::Write::write_all(&mut zw, b"image").unwrap();
        }
        zw.finish().unwrap();

        let config = GalleryConfig {
            zips_dir,
            expected_boards: vec![],
            wasm_dir: root.join("missing-wasm"),
            template,
            out: out.clone(),
        };
        generate(&config).unwrap();

        let index = fs::read_to_string(out.join("index.html")).unwrap();
        assert!(index.contains("screenshots/board/framebuffer-1.png"));
        assert!(index.contains("screenshots/board/snap-1.jpg"));

        fs::remove_dir_all(&root).unwrap();
    }
}
