use std::process::{Command, Stdio};
use std::path::Path;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    let dry_run = args.contains(&"--dry-run".to_string());
    let incompatible = args.contains(&"--incompatible".to_string());
    
    println!("🚀 ESP32 Embedded Projects Dependency Updater");
    println!("===============================================");
    if dry_run {
        println!("🔍 Running in DRY-RUN mode - no changes will be made");
    }
    if incompatible {
        println!("⚠️  Including incompatible updates (potentially breaking)");
    }
    println!();
    
    // List of all project directories
    let projects = vec![
        "esope-sld-c-w-s3",
        "esp32-c3-lcdkit", 
        "esp32-c6-memory-lcd",
        "esp32-s3-box-3",
        "esp32-s3-box-3-minimal",
        "esp32-s3-lcd-ev-board",
        "esp32-wrover-kit",
        "m5stack-atom-s3",
        "m5stack-cores3",
        "waveshare-esp32-c6-lcd-1_47",
        "waveshare-esp32-s3-touch-amoled-1_8",
        "waveshare-esp32-s3-touch-lcd-1_28",
    ];
    
    // Go up one directory to the project root
    let project_root = Path::new("../");
    
    let mut updated_count = 0;
    let mut failed_count = 0;
    let mut skipped_count = 0;
    
    for project in &projects {
        let project_path = project_root.join(project);
        let cargo_toml_path = project_path.join("Cargo.toml");
        
        println!("\n📦 Processing: {}", project);
        
        if !project_path.exists() {
            println!("⚠️  Directory not found: {}", project);
            skipped_count += 1;
            continue;
        }
        
        if !cargo_toml_path.exists() {
            println!("⚠️  Cargo.toml not found in: {}", project);
            skipped_count += 1;
            continue;
        }
        
        // Run cargo upgrade in the project directory
        let mut cmd = Command::new("cargo");
        cmd.arg("upgrade")
            .arg("--verbose")
            .current_dir(&project_path)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        
        if dry_run {
            cmd.arg("--dry-run");
        }
        if incompatible {
            cmd.arg("--incompatible");
        }
        
        let output = cmd.output();
        
        match output {
            Ok(result) => {
                if result.status.success() {
                    println!("✅ Successfully updated: {}", project);
                    // Print stdout if there's useful information
                    let stdout = String::from_utf8_lossy(&result.stdout);
                    if !stdout.trim().is_empty() {
                        for line in stdout.lines() {
                            if line.contains("Upgrading") || line.contains("Updated") || 
                               line.contains("incompatible") || line.contains("latest") {
                                println!("   {}", line);
                            }
                        }
                    }
                    updated_count += 1;
                } else {
                    println!("❌ Failed to update: {}", project);
                    let stderr = String::from_utf8_lossy(&result.stderr);
                    if !stderr.trim().is_empty() {
                        println!("   Error: {}", stderr.trim());
                    }
                    failed_count += 1;
                }
            }
            Err(e) => {
                println!("❌ Command execution error for {}: {}", project, e);
                failed_count += 1;
            }
        }
    }
    
    println!("\n===============================================");
    println!("📊 Update Summary:");
    println!("✅ Successfully updated: {} projects", updated_count);
    if failed_count > 0 {
        println!("❌ Failed: {} projects", failed_count);
    }
    if skipped_count > 0 {
        println!("⚠️  Skipped: {} projects", skipped_count);
    }
    
    if failed_count == 0 && skipped_count == 0 {
        println!("🎉 All projects updated successfully!");
    } else if updated_count > 0 {
        println!("⚡ Some projects were updated successfully!");
    }
    
    // Suggest next steps
    println!("\n💡 Next Steps:");
    if !dry_run {
        println!("1. Review the updated Cargo.toml files for any breaking changes");
        println!("2. Test build all projects: '../format_projects.sh'");
        println!("3. Run builds to ensure compatibility: 'cargo build --release' in each project");
    } else {
        println!("1. Review the above output to see what would be updated");
        println!("2. Run without --dry-run to apply updates");
        println!("3. Consider --incompatible flag for major version updates");
    }
    
    println!("\n📝 Usage:");
    println!("  cargo run --release                    # Update compatible versions");
    println!("  cargo run --release -- --dry-run       # Preview changes only");
    println!("  cargo run --release -- --incompatible  # Include breaking changes");
}
