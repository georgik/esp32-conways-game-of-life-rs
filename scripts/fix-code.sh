#!/bin/bash
set -euo pipefail

# This script formats the code using cargo fmt, runs cargo clippy with auto-fix,
# and if any files were modified, stages and commits them.

echo "Running cargo fmt to format code..."
cargo fmt --all

echo "Running cargo clippy with auto-fix (requires nightly)..."
cargo clippy --all-features --workspace --fix --allow-dirty

echo "Running tests..."
cargo test --all

