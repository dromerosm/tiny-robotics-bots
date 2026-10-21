#!/bin/sh
# Format all C sources, or check them with --check (used by CI).
# Needs clang-format 18.1.8:  pip install clang-format==18.1.8
set -eu
cd "$(dirname "$0")/.."
files="$(git ls-files '*.c' '*.h')"
[ -n "$files" ] || exit 0
if [ "${1:-}" = "--check" ]; then
  clang-format --dry-run --Werror $files
  echo "format: ok"
else
  clang-format -i $files
fi
