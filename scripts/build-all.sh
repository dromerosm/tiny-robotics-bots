#!/bin/sh
# Build every example. Stops at the first failure.
set -eu
cd "$(dirname "$0")/.."
for example in $(scripts/list-examples.sh); do
  scripts/build.sh "$example"
done
