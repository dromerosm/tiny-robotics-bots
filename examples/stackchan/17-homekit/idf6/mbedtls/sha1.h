// The HomeKit SDK asks for the mbedTLS 3 header. ESP-IDF 6 (mbedTLS 4) keeps it
// in private/. The build config goes first: it is what lets the private header
// declare its functions.
#include "tf-psa-crypto/build_info.h"
#include "mbedtls/private/sha1.h"
