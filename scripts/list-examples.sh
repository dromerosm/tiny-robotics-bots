#!/bin/sh
# Print every example folder (any folder under examples/ with a sdkconfig.defaults).
set -eu
cd "$(dirname "$0")/.."
find examples -mindepth 3 -maxdepth 3 -name sdkconfig.defaults -exec dirname {} \; | sort
