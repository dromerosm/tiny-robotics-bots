#!/bin/sh
# Build and run the host unit tests for the portable components (no ESP-IDF needed).
set -eu
cd "$(dirname "$0")/.."
mkdir -p .tools
for test in tests/host/test_*.c; do
  name="$(basename "$test" .c)"
  cc -std=c11 -Wall -Wextra -Werror -O2 -Icomponents/tr_pid/include \
     "$test" components/tr_pid/tr_pid.c -lm -o ".tools/$name"
  ".tools/$name"
done
