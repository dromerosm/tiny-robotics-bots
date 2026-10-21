#!/bin/sh
# Run an example's image in esp-emu and check its output.
#
#   scripts/smoke.sh examples/stackchan/01-hello
#
# The example's smoke.env says what to expect:
#   EMU_CHIP   chip for esp-emu (empty: the emulator does not support it, skip)
#   EXPECT     text that must appear in the output
#   INJECT_ON  optional: when this text appears, send INJECT to UART0
#   INJECT     optional: text to send (use \n for Enter)
#   TIMEOUT    optional: default 20s
# esp-emu exits 0 even on timeout, so this script checks the output itself.
set -eu
cd "$(dirname "$0")/.."

EXAMPLE="${1:?usage: scripts/smoke.sh examples/<robot>/<example>}"
EXAMPLE="${EXAMPLE%/}"
[ -f "$EXAMPLE/smoke.env" ] || { echo "skip: $EXAMPLE has no smoke.env"; exit 0; }
EMU_CHIP="" EXPECT="" INJECT_ON="" INJECT="" TIMEOUT="20s"
. "$EXAMPLE/smoke.env"
[ -n "$EMU_CHIP" ] || { echo "skip: $EXAMPLE (chip not supported by esp-emu)"; exit 0; }
[ -n "$EXPECT" ] || { echo "error: EXPECT is empty in $EXAMPLE/smoke.env" >&2; exit 1; }

TARGET="$(sed -n 's/^CONFIG_IDF_TARGET="\(.*\)"/\1/p' "$EXAMPLE/sdkconfig.defaults")"
IMAGE="out/$(basename "$(dirname "$EXAMPLE")")-$(basename "$EXAMPLE")-$TARGET.bin"
[ -f "$IMAGE" ] || { echo "error: $IMAGE not found, build it first" >&2; exit 1; }
EMU="${ESP_EMU:-.tools/bin/esp-emu}"

set -- --chip "$EMU_CHIP" --firmware "$IMAGE" --timeout "$TIMEOUT" --exit-on "$EXPECT"
[ -n "$INJECT_ON" ] && set -- "$@" --inject "$INJECT" --inject-on "$INJECT_ON"

log="$(mktemp)"
trap 'rm -f "$log"' EXIT
"$EMU" "$@" >"$log" 2>&1 || true
if grep -qF -- "$EXPECT" "$log"; then
  echo "pass: $EXAMPLE"
else
  echo "FAIL: $EXAMPLE did not print \"$EXPECT\" within $TIMEOUT. Last lines:" >&2
  tail -n 25 "$log" >&2
  exit 1
fi
