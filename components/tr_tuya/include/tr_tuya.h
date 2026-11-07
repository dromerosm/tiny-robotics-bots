// tr_tuya: switch Tuya (Smart Life) plugs on the home network, with no cloud.
//
// It speaks the local protocol on TCP port 6668: version 3.3 (AES with the
// device's local key) and 3.4 (a session key agreed first, then AES and
// HMAC-SHA256). Every call opens a connection, does one thing and closes it:
// some devices (our power strip) take only one connection at a time, and a
// phone app may want it.
//
// Portable C with BSD sockets: the same file builds on the robot (lwIP) and on
// a computer (tests/host). The crypto comes from tr_tuya_crypto.h, written once
// for the robot (PSA, tr_tuya_esp.c) and once for the host tests.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *id;  // the device id (gwId), 20 or 22 characters
    const char *ip;  // "192.168.1.33"
    uint8_t key[16]; // the local key
    int version;     // 33 or 34
} tr_tuya_dev_t;

// One data point from a status: dp number and value (true is 1, false 0).
// Text values are left out.
typedef struct {
    int dp;
    int value;
} tr_tuya_dp_t;

// Error codes (all negative).
#define TR_TUYA_ERR_CONNECT -1 // no TCP connection (device off, wrong IP, or busy)
#define TR_TUYA_ERR_IO      -2 // the connection broke or timed out
#define TR_TUYA_ERR_FRAME   -3 // a bad answer: wrong key or wrong version
#define TR_TUYA_ERR_SESSION -4 // the 3.4 session key was refused (wrong key)
#define TR_TUYA_ERR_ARG     -5

// The device's data points, up to max. Returns how many, or an error.
int tr_tuya_status(const tr_tuya_dev_t *dev, tr_tuya_dp_t *dps, int max);

// Switch one data point on or off. Returns 0 when the device took it.
int tr_tuya_set(const tr_tuya_dev_t *dev, int dp, bool on);

// The value of dp in a status, or -1 when it is not there.
int tr_tuya_value(const tr_tuya_dp_t *dps, int count, int dp);

// A short text for an error code.
const char *tr_tuya_err_text(int err);

// --- The protocol pieces, for the host tests -------------------------------

// A frame: 0x000055AA, seq, cmd, length, payload, CRC32 (or HMAC-SHA256 when
// hmac_key is set), 0x0000AA55. Returns its size, or 0 when out is too small.
size_t tr_tuya_frame(uint8_t *out, size_t cap, uint32_t seq, uint32_t cmd, const uint8_t *payload,
                     size_t len, const uint8_t *hmac_key);

// Read one frame that a device sent (it has a 4-byte return code first).
// Returns the bytes it used, 0 when more bytes are needed, or an error.
// payload points into in, after the return code.
int tr_tuya_unframe(const uint8_t *in, size_t len, const uint8_t *hmac_key, uint32_t *cmd,
                    const uint8_t **payload, size_t *plen);

// AES-128-ECB with PKCS#7 padding. encrypt returns the new size (out needs
// len + 16 bytes). decrypt returns the size without padding, or an error.
int tr_tuya_encrypt(const uint8_t key[16], const uint8_t *in, size_t len, uint8_t *out);
int tr_tuya_decrypt(const uint8_t key[16], const uint8_t *in, size_t len, uint8_t *out);

// The 3.4 session key: AES-ECB(local key, local nonce XOR remote nonce).
int tr_tuya_session_key(const uint8_t key[16], const uint8_t local_nonce[16],
                        const uint8_t remote_nonce[16], uint8_t session[16]);

// Read the "dps":{...} object of a JSON answer. Returns how many (up to max).
int tr_tuya_parse_dps(const char *json, tr_tuya_dp_t *dps, int max);

// Parse 32 hex characters into a key. Returns false on bad input.
bool tr_tuya_key_from_hex(const char *hex, uint8_t key[16]);

#ifdef __cplusplus
}
#endif
