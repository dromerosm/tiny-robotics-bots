#!/bin/sh
# Build one example into a merged image you can flash or drop in the emulator.
#
#   scripts/build.sh examples/stackchan/01-hello
#
# Output: out/<robot>-<example>-<chip>.bin (for example out/stackchan-01-hello-esp32s3.bin)
# Uses ESP-IDF from Docker (image pinned in tools.env), or the local ESP-IDF
# when IDF_PATH is set (as in CI's ESP-IDF container).
set -eu
cd "$(dirname "$0")/.."
. ./tools.env

EXAMPLE="${1:?usage: scripts/build.sh examples/<robot>/<example>}"
EXAMPLE="${EXAMPLE%/}"
[ -f "$EXAMPLE/CMakeLists.txt" ] || { echo "error: not an ESP-IDF project: $EXAMPLE" >&2; exit 1; }
TARGET="$(sed -n 's/^CONFIG_IDF_TARGET="\(.*\)"/\1/p' "$EXAMPLE/sdkconfig.defaults" 2>/dev/null)"
[ -n "$TARGET" ] || { echo "error: set CONFIG_IDF_TARGET in $EXAMPLE/sdkconfig.defaults" >&2; exit 1; }

ROBOT="$(basename "$(dirname "$EXAMPLE")")"
NAME="$ROBOT-$(basename "$EXAMPLE")-$TARGET"

BUILD="idf.py -C '$EXAMPLE' -B '$EXAMPLE/build' set-target $TARGET && idf.py -C '$EXAMPLE' -B '$EXAMPLE/build' build && idf.py -C '$EXAMPLE' -B '$EXAMPLE/build' merge-bin"
if [ -n "${IDF_PATH:-}" ]; then
  sh -c "$BUILD"
else
  docker run --rm -v "$PWD:/project" -w /project "$IDF_IMAGE" bash -c "$BUILD"
fi

mkdir -p out
cp "$EXAMPLE/build/merged-binary.bin" "out/$NAME.bin"
echo "built out/$NAME.bin"
