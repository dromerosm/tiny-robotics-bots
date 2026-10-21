#!/bin/sh
# Build one project and write a merged image you can drop in the emulator.
# Usage: scripts/build.sh <project dir>   e.g. scripts/build.sh stackchan/hello
# Output: out/<robot>-<project>-<target>.bin
# Runs ESP-IDF in Docker, unless IDF_PATH is set (as in CI's IDF container).
set -eu
cd "$(dirname "$0")/.."
. ./idf.env

PROJECT="${1:?usage: scripts/build.sh <project dir>}"
[ -f "$PROJECT/CMakeLists.txt" ] || { echo "not an ESP-IDF project: $PROJECT" >&2; exit 1; }
TARGET="$(sed -n 's/^CONFIG_IDF_TARGET="\(.*\)"/\1/p' "$PROJECT/sdkconfig.defaults" 2>/dev/null)"
[ -n "$TARGET" ] || { echo "set CONFIG_IDF_TARGET in $PROJECT/sdkconfig.defaults" >&2; exit 1; }
NAME="$(echo "$PROJECT" | tr '/' '-')-$TARGET"

BUILD="idf.py -C '$PROJECT' set-target $TARGET && idf.py -C '$PROJECT' build && idf.py -C '$PROJECT' merge-bin"
if [ -n "${IDF_PATH:-}" ]; then
  sh -c "$BUILD"
else
  docker run --rm -v "$PWD:/project" -w /project "$IDF_IMAGE" bash -c "$BUILD"
fi

mkdir -p out
cp "$PROJECT/build/merged-binary.bin" "out/$NAME.bin"
ls -lh "out/$NAME.bin"
