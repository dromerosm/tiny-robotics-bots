#!/bin/sh
# Install the pinned esp-emu (Espressif's ESP32 emulator) into .tools/bin.
# The official installer checks the download against the release SHA256SUMS.
set -eu
cd "$(dirname "$0")/.."
. ./tools.env
mkdir -p .tools/bin
curl -fsSL "https://raw.githubusercontent.com/espressif/esp-emulator/v${ESP_EMU_VERSION}/install.sh" \
  | sh -s -- --version "$ESP_EMU_VERSION" --bin-dir "$PWD/.tools/bin" --quiet
.tools/bin/esp-emu --version
