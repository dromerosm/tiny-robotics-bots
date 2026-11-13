#!/bin/sh
# Test and build the virtual parts in parts/ (Rust) into WebAssembly modules
# for esp-emu: out/<name>.wasm. The emulator runs them next to the firmware;
# parts/stackchan-body plays StackChan's body (the PY32 and the head servos).
#
#   scripts/build-parts.sh
#
# Uses the local Rust toolchain when it has the wasm32 target (as in CI),
# otherwise the Rust Docker image. Both use RUST_VERSION from tools.env.
set -eu
cd "$(dirname "$0")/.."
. ./tools.env

TARGET=wasm32-unknown-unknown
mkdir -p out
for dir in parts/*/; do
  name="$(basename "$dir")"
  crate="$(echo "$name" | tr - _)"
  if command -v rustup >/dev/null 2>&1 && rustup +"$RUST_VERSION" target list --installed 2>/dev/null | grep -qx "$TARGET"; then
    (cd "$dir" && cargo +"$RUST_VERSION" test --release --locked -q && cargo +"$RUST_VERSION" build --release --locked --target "$TARGET")
  else
    docker run --rm -v "$PWD/$dir:/part" -w /part "rust:$RUST_VERSION-slim" \
      sh -c "rustup target add $TARGET >/dev/null && cargo test --release --locked -q && cargo build --release --locked --target $TARGET"
  fi
  cp "$dir/target/$TARGET/release/$crate.wasm" "out/$name.wasm"
  echo "built out/$name.wasm ($(wc -c < "out/$name.wasm" | tr -d ' ') bytes)"
done
