#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_dir="${QUALISYS_SDK_DIR:-"${repo_root}/../qualisys_rust_sdk"}"

if [[ ! -f "${sdk_dir}/Cargo.toml" ]]; then
  echo "QUALISYS_SDK_DIR does not point to a qualisys_rust_sdk checkout: ${sdk_dir}" >&2
  exit 1
fi

cd "${repo_root}"

cargo fmt --all -- --check
cargo clippy --locked --all-targets
cargo test --locked
cargo build --locked --bin synapse-qualisys-bridge

cargo build --manifest-path "${sdk_dir}/Cargo.toml" --bin qualisys-sim

export BRIDGE_BIN="${repo_root}/target/debug/synapse-qualisys-bridge"
export QUALISYS_SIM_BIN="${sdk_dir}/target/debug/qualisys-sim"

playwright test --config tests/e2e/playwright.config.js
