// The crypto tr_tuya needs, written once per platform: PSA on the robot
// (tr_tuya_esp.c), CommonCrypto or OpenSSL in the host tests.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// AES-128-ECB, no padding. len is a multiple of 16. Returns 0 on success.
int tr_tuya_aes_ecb(const uint8_t key[16], bool encrypt, const uint8_t *in, uint8_t *out,
                    size_t len);

// HMAC-SHA256 with a 16-byte key. Returns 0 on success.
int tr_tuya_hmac(const uint8_t key[16], const uint8_t *data, size_t len, uint8_t out[32]);

// Random bytes, for the 3.4 nonce.
void tr_tuya_random(uint8_t *out, size_t len);

// One call at a time, across tasks (the screen, the voice and the web server
// may all switch something).
void tr_tuya_lock(void);
void tr_tuya_unlock(void);
