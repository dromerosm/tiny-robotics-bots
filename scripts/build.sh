#!/bin/sh
# Build one example into a merged image you can flash or drop in the emulator.
#
#   scripts/build.sh examples/stackchan/01-hello
#
# Output: out/<robot>-<example>-<chip>.bin (for example out/stackchan-01-hello-esp32s3.bin)
# Uses ESP-IDF from Docker (image pinned in tools.env), or the local ESP-IDF
# when IDF_PATH is set (as in CI's ESP-IDF container).
#
# Optional: WIFI_ENV=path/to/wifi.env passes the file's KEY=value lines to the
# build as environment variables (05-ota takes the home Wi-Fi from them). The
# file is read where it is, so passwords never go into this repo.
set -eu
if [ -n "${WIFI_ENV:-}" ]; then
  [ -f "$WIFI_ENV" ] || { echo "error: WIFI_ENV file not found: $WIFI_ENV" >&2; exit 1; }
  WIFI_ENV="$(cd "$(dirname "$WIFI_ENV")" && pwd)/$(basename "$WIFI_ENV")"
fi
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
  if [ -n "${WIFI_ENV:-}" ]; then
    (set -a && . "$WIFI_ENV" && sh -c "$BUILD")
  else
    sh -c "$BUILD"
  fi
elif [ -n "${WIFI_ENV:-}" ]; then
  docker run --rm --env-file "$WIFI_ENV" -v "$PWD:/project" -w /project "$IDF_IMAGE" bash -c "$BUILD"
else
  docker run --rm -v "$PWD:/project" -w /project "$IDF_IMAGE" bash -c "$BUILD"
fi

mkdir -p out
cp "$EXAMPLE/build/merged-binary.bin" "out/$NAME.bin"
echo "built out/$NAME.bin"
