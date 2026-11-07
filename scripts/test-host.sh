#!/bin/sh
# Build and run the host unit tests for the portable components (no ESP-IDF needed).
set -eu
cd "$(dirname "$0")/.."
mkdir -p .tools
# tr_tuya's crypto: CommonCrypto on macOS (part of the system), OpenSSL elsewhere.
CRYPTO_LIB=""
[ "$(uname)" = "Darwin" ] || CRYPTO_LIB="-lcrypto"
for test in tests/host/test_*.c; do
  name="$(basename "$test" .c)"
  case "$name" in
    test_pid)  extra="components/tr_pid/tr_pid.c" ; libs="-lm" ;;
    test_tuya) extra="components/tr_tuya/tr_tuya.c tests/host/tuya_crypto_host.c" ; libs="$CRYPTO_LIB" ;;
    *)         extra="" ; libs="" ;;
  esac
  # shellcheck disable=SC2086
  cc -std=c11 -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE -Wall -Wextra -Werror -O2 \
     -Icomponents/tr_pid/include -Icomponents/tr_tuya/include \
     "$test" $extra $libs -o ".tools/$name"
  ".tools/$name"
done
