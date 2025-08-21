use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::fs;
use std::path::PathBuf;
use std::process::Stdio;
use tokio::process::Command as TokioCommand;

#[derive(Parser)]
#[command(name = "xtask")]
#[command(about = "ESP32 Embedded Projects Maintenance Tool")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Build all ESP32 projects with release profile
    Build {
        /// Continue building other projects even if one fails
        #[arg(long)]
        keep_going: bool,
        /// Show verbose output from cargo builds
        #[arg(long, short)]
        verbose: bool,
    },
    /// Update dependencies across all projects
    Update {
        /// Preview changes without applying them
        #[arg(long)]
        dry_run: bool,
        /// Include incompatible/breaking version updates
        #[arg(long)]
        incompatible: bool,
        /// Show verbose output
        #[arg(long, short)]
        verbose: bool,
    },
    /// Format all project code using cargo fmt
    Format {
        /// Show verbose output
        #[arg(long, short)]
        verbose: bool,
    },
    /// Run clippy on all projects
    Clippy {
        /// Show verbose output
        #[arg(long, short)]
        verbose: bool,
    },
    /// Run all maintenance tasks: format, update (compatible only), and build
    All {
        /// Continue with other tasks even if one fails
        #[arg(long)]
        keep_going: bool,
        /// Show verbose output
        #[arg(long, short)]
        verbose: bool,
    },
    /// List all discovered ESP32 projects
    List,
    /// Fix workspace issues by adding empty [workspace] to all projects
    FixWorkspace,
}

// Project configuration
const PROJECT_PATTERNS: &[&str] = &["esope*", "esp32*", "m5stack*", "waveshare*"];

struct ProjectInfo {
    name: String,
    path: PathBuf,
    has_cargo_toml: bool,
}

struct TaskResult {
    project: String,
    success: bool,
    message: String,
    warnings: Vec<String>,
}

struct TaskSummary {
    total: usize,
    success: usize,
    failed: usize,
    warnings: usize,
}

impl TaskSummary {
    fn new() -> Self {
        Self {
            total: 0,
            success: 0,
            failed: 0,
            warnings: 0,
        }
    }

    fn add_result(&mut self, result: &TaskResult) {
        self.total += 1;
        if result.success {
            self.success += 1;
        } else {
            self.failed += 1;
        }
        self.warnings += result.warnings.len();
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    // Print banner
    println!("🚀 ESP32 Embedded Projects Maintenance Tool");
    println!("===========================================");

    // Discover projects
    let projects = discover_projects().await?;
    if projects.is_empty() {
        println!(
            "⚠️  No ESP32 projects found matching patterns: {:?}",
            PROJECT_PATTERNS
        );
        return Ok(());
    }

    match cli.command {
        Commands::List => list_projects(&projects).await,
        Commands::Build {
            keep_going,
            verbose,
        } => build_all_projects(&projects, keep_going, verbose).await,
        Commands::Update {
            dry_run,
            incompatible,
            verbose,
        } => update_dependencies(&projects, dry_run, incompatible, verbose).await,
        Commands::Format { verbose } => format_all_projects(&projects, verbose).await,
        Commands::Clippy { verbose } => clippy_all_projects(&projects, verbose).await,
        Commands::All {
            keep_going,
            verbose,
        } => run_all_tasks(&projects, keep_going, verbose).await,
        Commands::FixWorkspace => fix_workspace_issues(&projects).await,
    }
}

async fn discover_projects() -> Result<Vec<ProjectInfo>> {
    let current_dir = std::env::current_dir()?;
    let mut projects = Vec::new();

    // Read directory entries
    let mut entries = fs::read_dir(&current_dir).context("Failed to read current directory")?;

    while let Some(entry) = entries.next().transpose()? {
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }

        let dir_name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or_default();

        // Check if directory matches any of our patterns
        let matches_pattern = PROJECT_PATTERNS.iter().any(|pattern| {
            let pattern = pattern.trim_end_matches('*');
            dir_name.starts_with(pattern)
        });

        if matches_pattern {
            let cargo_toml = path.join("Cargo.toml");
            let has_cargo_toml = cargo_toml.exists();

            projects.push(ProjectInfo {
                name: dir_name.to_string(),
                path,
                has_cargo_toml,
            });
        }
    }

    // Sort projects by name for consistent output
    projects.sort_by(|a, b| a.name.cmp(&b.name));

    Ok(projects)
}

async fn list_projects(projects: &[ProjectInfo]) -> Result<()> {
    println!("\n📋 Discovered ESP32 Projects ({} total):", projects.len());
    println!("┌─────┬──────────────────────────────────────────────┬────────────┐");
    println!("│ #   │ Project Name                                 │ Status     │");
    println!("├─────┼──────────────────────────────────────────────┼────────────┤");

    for (i, project) in projects.iter().enumerate() {
        let status = if project.has_cargo_toml {
            "✅ Ready"
        } else {
            "❌ No Cargo.toml"
        };
        println!("│ {:3} │ {:<44} │ {} │", i + 1, project.name, status);
    }

    println!("└─────┴──────────────────────────────────────────────┴────────────┘");

    let ready_count = projects.iter().filter(|p| p.has_cargo_toml).count();
    println!(
        "\n📊 Summary: {} ready for build, {} missing Cargo.toml",
        ready_count,
        projects.len() - ready_count
    );

    Ok(())
}

async fn build_all_projects(
    projects: &[ProjectInfo],
    keep_going: bool,
    verbose: bool,
) -> Result<()> {
    println!("\n🔨 Building all ESP32 projects with --release profile");
    println!("════════════════════════════════════════════════════");

    let mut summary = TaskSummary::new();
    let mut results = Vec::new();

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        println!("\n🔨 Building: {}", project.name);

        let result = build_project(project, verbose).await?;

        if result.success {
            println!("✅ Build successful: {}", project.name);
            if !result.warnings.is_empty() {
                println!("⚠️  {} warnings found:", result.warnings.len());
                for warning in &result.warnings[..std::cmp::min(5, result.warnings.len())] {
                    println!("   {}", warning);
                }
                if result.warnings.len() > 5 {
                    println!("   ... and {} more warnings", result.warnings.len() - 5);
                }
            }
        } else {
            println!("❌ Build failed: {}", project.name);
            // Always show error details for failures, regardless of verbose flag
            println!("   Error details:");
            for line in result.message.lines().take(20) {
                println!("   {}", line);
            }
            if result.message.lines().count() > 20 {
                println!("   ... (truncated, use --verbose for full output)");
            }
            if !keep_going {
                return Err(anyhow::anyhow!("Build failed for {}", project.name));
            }
        }

        summary.add_result(&result);
        results.push(result);
    }

    print_build_summary(&summary, &results).await;

    if summary.failed > 0 && !keep_going {
        std::process::exit(1);
    }

    Ok(())
}

async fn build_project(project: &ProjectInfo, verbose: bool) -> Result<TaskResult> {
    // Read the rust-toolchain.toml to determine which toolchain to use
    let toolchain_path = project.path.join("rust-toolchain.toml");
    let toolchain = if toolchain_path.exists() {
        let content =
            fs::read_to_string(&toolchain_path).context("Failed to read rust-toolchain.toml")?;

        // Parse the toolchain channel from TOML
        if content.contains("channel = \"esp\"") {
            "esp"
        } else if content.contains("channel = \"stable\"") {
            "stable"
        } else {
            "stable" // fallback
        }
    } else {
        "stable" // fallback if no toolchain file
    };

    let output = TokioCommand::new("rustup")
        .arg("run")
        .arg(toolchain)
        .arg("cargo")
        .arg("build")
        .arg("--release")
        .current_dir(&project.path)
        .output()
        .await
        .context("Failed to run cargo build")?;

    let success = output.status.success();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    let mut warnings = Vec::new();
    let combined_output = format!("{}{}", stdout, stderr);

    // Extract warnings from output
    for line in combined_output.lines() {
        if verbose {
            println!("   {}", line);
        }
        if line.contains("warning:") {
            warnings.push(line.to_string());
        }
    }

    let message = if success {
        "Built successfully".to_string()
    } else {
        format!("{}{}", stdout, stderr)
    };

    Ok(TaskResult {
        project: project.name.clone(),
        success,
        message,
        warnings,
    })
}

async fn update_dependencies(
    projects: &[ProjectInfo],
    dry_run: bool,
    incompatible: bool,
    verbose: bool,
) -> Result<()> {
    println!("\n📦 Updating dependencies across all projects");
    if dry_run {
        println!("🔍 Running in DRY-RUN mode - no changes will be made");
    }
    if incompatible {
        println!("⚠️  Including incompatible updates (potentially breaking)");
    }
    println!("═════════════════════════════════════════════════");

    let mut summary = TaskSummary::new();

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        println!("\n📦 Processing: {}", project.name);

        let result = update_project_deps(project, dry_run, incompatible, verbose).await?;

        if result.success {
            println!("✅ Dependencies updated: {}", project.name);
        } else {
            println!("❌ Update failed: {}", project.name);
            if verbose {
                println!("   Error: {}", result.message);
            }
        }

        summary.add_result(&result);
    }

    println!("\n═════════════════════════════════════════════════");
    println!("📊 Update Summary:");
    println!("✅ Successfully updated: {} projects", summary.success);
    if summary.failed > 0 {
        println!("❌ Failed: {} projects", summary.failed);
    }

    if summary.success > 0 {
        println!("\n💡 Next Steps:");
        if !dry_run {
            println!("1. Review updated Cargo.toml files for breaking changes");
            println!("2. Test builds: cargo xtask build");
            println!("3. Verify functionality of updated projects");
        } else {
            println!("1. Review the above output");
            println!("2. Run without --dry-run to apply updates");
            println!("3. Consider --incompatible for major version updates");
        }
    }

    Ok(())
}

async fn update_project_deps(
    project: &ProjectInfo,
    dry_run: bool,
    incompatible: bool,
    verbose: bool,
) -> Result<TaskResult> {
    let mut cmd = TokioCommand::new("cargo");
    cmd.arg("upgrade")
        .current_dir(&project.path)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    if dry_run {
        cmd.arg("--dry-run");
    }
    if incompatible {
        cmd.arg("--incompatible");
    }
    if verbose {
        cmd.arg("--verbose");
    }

    let output = cmd.output().await.context("Failed to run cargo upgrade")?;

    let success = output.status.success();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Extract useful information from output
    let mut message_lines = Vec::new();
    for line in stdout.lines().chain(stderr.lines()) {
        if line.contains("Upgrading")
            || line.contains("Updated")
            || line.contains("incompatible")
            || line.contains("latest")
            || line.trim().starts_with("name")
            || line.contains("->")
        {
            message_lines.push(line.to_string());
            if verbose {
                println!("   {}", line);
            }
        }
    }

    let message = if success {
        "Dependencies processed successfully".to_string()
    } else {
        stderr.to_string()
    };

    Ok(TaskResult {
        project: project.name.clone(),
        success,
        message,
        warnings: Vec::new(),
    })
}

async fn format_all_projects(projects: &[ProjectInfo], verbose: bool) -> Result<()> {
    println!("\n🎨 Formatting all ESP32 projects");
    println!("═══════════════════════════════");

    let mut summary = TaskSummary::new();

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        println!("\n🎨 Formatting: {}", project.name);

        let result = format_project(project, verbose).await?;

        if result.success {
            println!("✅ Formatted successfully: {}", project.name);
        } else {
            println!("❌ Format failed: {}", project.name);
            if verbose {
                println!("   Error: {}", result.message);
            }
        }

        summary.add_result(&result);
    }

    println!("\n═══════════════════════════════");
    println!("📊 Format Summary:");
    println!("✅ Successfully formatted: {} projects", summary.success);
    if summary.failed > 0 {
        println!("❌ Failed: {} projects", summary.failed);
    }

    Ok(())
}

async fn format_project(project: &ProjectInfo, verbose: bool) -> Result<TaskResult> {
    // Read the rust-toolchain.toml to determine which toolchain to use
    let toolchain_path = project.path.join("rust-toolchain.toml");
    let toolchain = if toolchain_path.exists() {
        let content =
            fs::read_to_string(&toolchain_path).context("Failed to read rust-toolchain.toml")?;

        // Parse the toolchain channel from TOML
        if content.contains("channel = \"esp\"") {
            "esp"
        } else if content.contains("channel = \"stable\"") {
            "stable"
        } else {
            "stable" // fallback
        }
    } else {
        "stable" // fallback if no toolchain file
    };

    let output = TokioCommand::new("rustup")
        .arg("run")
        .arg(toolchain)
        .arg("cargo")
        .arg("fmt")
        .current_dir(&project.path)
        .output()
        .await
        .context("Failed to run cargo fmt")?;

    let success = output.status.success();
    let stderr = String::from_utf8_lossy(&output.stderr);

    if verbose && !stderr.is_empty() {
        println!("   {}", stderr);
    }

    Ok(TaskResult {
        project: project.name.clone(),
        success,
        message: if success {
            "Formatted".to_string()
        } else {
            stderr.to_string()
        },
        warnings: Vec::new(),
    })
}

async fn clippy_all_projects(projects: &[ProjectInfo], verbose: bool) -> Result<()> {
    println!("\n🔍 Running clippy on all ESP32 projects (--release mode)");
    println!("════════════════════════════════════════════════════");

    let mut summary = TaskSummary::new();

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        println!("\n🔍 Clippy: {}", project.name);

        let result = clippy_project(project, verbose).await?;

        if result.success {
            println!("✅ Clippy passed: {}", project.name);
            if !result.warnings.is_empty() {
                println!("⚠️  {} clippy warnings found:", result.warnings.len());
                for warning in &result.warnings[..std::cmp::min(3, result.warnings.len())] {
                    println!("   {}", warning);
                }
                if result.warnings.len() > 3 {
                    println!("   ... and {} more warnings", result.warnings.len() - 3);
                }
            }
        } else {
            println!("❌ Clippy failed: {}", project.name);
            // Always show error details for failures, regardless of verbose flag
            println!("   Error details:");
            for line in result.message.lines().take(20) {
                println!("   {}", line);
            }
            if result.message.lines().count() > 20 {
                println!("   ... (truncated, use --verbose for full output)");
            }
        }

        summary.add_result(&result);
    }

    println!("\n═══════════════════════════════════════");
    println!("📊 Clippy Summary:");
    println!("✅ Clippy passed: {} projects", summary.success);
    if summary.failed > 0 {
        println!("❌ Clippy failed: {} projects", summary.failed);
    }
    if summary.warnings > 0 {
        println!("⚠️  Total clippy warnings: {}", summary.warnings);
    }

    Ok(())
}

async fn clippy_project(project: &ProjectInfo, verbose: bool) -> Result<TaskResult> {
    // Read the rust-toolchain.toml to determine which toolchain to use
    let toolchain_path = project.path.join("rust-toolchain.toml");
    let toolchain = if toolchain_path.exists() {
        let content =
            fs::read_to_string(&toolchain_path).context("Failed to read rust-toolchain.toml")?;

        // Parse the toolchain channel from TOML
        if content.contains("channel = \"esp\"") {
            "esp"
        } else if content.contains("channel = \"stable\"") {
            "stable"
        } else {
            "stable" // fallback
        }
    } else {
        "stable" // fallback if no toolchain file
    };

    let output = TokioCommand::new("rustup")
        .arg("run")
        .arg(toolchain)
        .arg("cargo")
        .arg("clippy")
        .arg("--release")
        .arg("--all-features")
        .arg("--workspace")
        .arg("--")
        .arg("-D")
        .arg("warnings")
        .current_dir(&project.path)
        .output()
        .await
        .context("Failed to run cargo clippy")?;

    let success = output.status.success();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    let mut warnings = Vec::new();
    let combined_output = format!("{}{}", stdout, stderr);

    // Extract warnings from output
    for line in combined_output.lines() {
        if verbose {
            println!("   {}", line);
        }
        if line.contains("warning:") || line.contains("help:") {
            warnings.push(line.to_string());
        }
    }

    let message = if success {
        "Clippy passed".to_string()
    } else {
        format!("{}{}", stdout, stderr)
    };

    Ok(TaskResult {
        project: project.name.clone(),
        success,
        message,
        warnings,
    })
}

async fn run_all_tasks(projects: &[ProjectInfo], keep_going: bool, verbose: bool) -> Result<()> {
    println!("\n🚀 Running ALL maintenance tasks");
    println!("═══════════════════════════════");
    println!("Tasks: Format → Update (compatible) → Build");

    // Step 1: Format
    println!("\n📝 Step 1/3: Formatting code...");
    format_all_projects(projects, verbose).await?;

    // Step 2: Update compatible dependencies
    println!("\n📦 Step 2/3: Updating compatible dependencies...");
    update_dependencies(projects, false, false, verbose).await?;

    // Step 3: Build all
    println!("\n🔨 Step 3/3: Building all projects...");
    build_all_projects(projects, keep_going, verbose).await?;

    println!("\n🎉 All maintenance tasks completed!");
    Ok(())
}

async fn print_build_summary(summary: &TaskSummary, results: &[TaskResult]) {
    println!("\n═════════════════════════════════════════════════");
    println!("📊 Build Summary:");
    println!("✅ Successfully built: {} projects", summary.success);
    if summary.failed > 0 {
        println!("❌ Failed to build: {} projects", summary.failed);
        println!("\nFailed projects:");
        for result in results.iter().filter(|r| !r.success) {
            println!("  • {}", result.project);
        }
    }
    if summary.warnings > 0 {
        println!("⚠️  Total warnings: {}", summary.warnings);
        println!("\nProjects with warnings:");
        for result in results.iter().filter(|r| !r.warnings.is_empty()) {
            println!(
                "  • {} ({} warnings)",
                result.project,
                result.warnings.len()
            );
        }
    }

    if summary.failed == 0 && summary.warnings == 0 {
        println!("🎉 All projects built successfully with no warnings!");
    } else if summary.failed == 0 {
        println!("✨ All projects built successfully (with some warnings)");
    }
}

async fn fix_workspace_issues(projects: &[ProjectInfo]) -> Result<()> {
    println!("\n🔧 Fixing workspace issues in all ESP32 projects");
    println!("═══════════════════════════════════════════════");

    let mut fixed_count = 0;
    let mut skipped_count = 0;

    for project in projects.iter().filter(|p| p.has_cargo_toml) {
        let cargo_toml_path = project.path.join("Cargo.toml");

        println!("\n🔧 Processing: {}", project.name);

        // Read current Cargo.toml
        let content = match fs::read_to_string(&cargo_toml_path) {
            Ok(content) => content,
            Err(e) => {
                println!("❌ Failed to read Cargo.toml: {}", e);
                continue;
            }
        };

        // Check if it already has a [workspace] section
        if content.contains("[workspace]") {
            println!("⏭️  Already has [workspace] section, skipping");
            skipped_count += 1;
            continue;
        }

        // Find the [package] section and add [workspace] after it
        let lines: Vec<&str> = content.lines().collect();
        let mut new_lines = Vec::new();
        let mut added_workspace = false;

        for line in lines {
            new_lines.push(line.to_string());

            // After the [package] section, add empty [workspace]
            if !added_workspace && line.starts_with("[package]") {
                // Find the end of the [package] section
                continue;
            }

            // Look for the first section after [package] or dependencies
            if !added_workspace && line.starts_with("[") && !line.starts_with("[package]") {
                // Insert empty workspace before this section
                new_lines.insert(new_lines.len() - 1, String::new());
                new_lines.insert(
                    new_lines.len() - 1,
                    "# Empty workspace to avoid being part of parent workspace".to_string(),
                );
                new_lines.insert(new_lines.len() - 1, "[workspace]".to_string());
                new_lines.insert(new_lines.len() - 1, String::new());
                added_workspace = true;
            }
        }

        // If we didn't find another section, add at the end
        if !added_workspace {
            new_lines.push(String::new());
            new_lines.push("# Empty workspace to avoid being part of parent workspace".to_string());
            new_lines.push("[workspace]".to_string());
            new_lines.push(String::new());
        }

        let new_content = new_lines.join("\n");

        // Write back to file
        match fs::write(&cargo_toml_path, new_content) {
            Ok(_) => {
                println!("✅ Added empty [workspace] section");
                fixed_count += 1;
            }
            Err(e) => {
                println!("❌ Failed to write Cargo.toml: {}", e);
            }
        }
    }

    println!("\n═══════════════════════════════════════════════");
    println!("📊 Workspace Fix Summary:");
    println!("✅ Fixed: {} projects", fixed_count);
    if skipped_count > 0 {
        println!(
            "⏭️  Skipped: {} projects (already had [workspace])",
            skipped_count
        );
    }

    if fixed_count > 0 {
        println!("\n💡 All projects should now build independently!");
        println!("Try: cargo xtask build");
    }

    Ok(())
}
