# tr_tuya

Switches Tuya (Smart Life, GoSound) plugs on the home network, with no cloud. It speaks Tuya's local protocol on TCP port 6668, versions **3.3** and **3.4**, and reads the device's data points (what is on, the power, the voltage).

```c
#include "tr_tuya.h"

tr_tuya_dev_t plug = {.id = "bf79...", .ip = "192.168.1.36", .version = 34};
tr_tuya_key_from_hex(key_hex, plug.key);

tr_tuya_dp_t dps[24];
int n = tr_tuya_status(&plug, dps, 24);           // dps, or an error (< 0)
int watts10 = tr_tuya_value(dps, n, 19);          // power, in tenths of a watt
int r = tr_tuya_set(&plug, 1, true);              // dp 1 (the outlet) on
printf("%s\n", tr_tuya_err_text(r));
```

## What each device needs

- **id** (gwId), **IP** and **version**: the devices broadcast them on UDP 6667 every few seconds. `tr_tuya_broadcast()` reads one of those datagrams and gives the id and the IP, so a robot can follow a device when its address changes ([17-homekit](../../examples/stackchan/17-homekit) does).
- **local key**: 16 bytes, from Tuya's cloud. It changes when the device is paired again.

Both come from `~/GitHub/home-iot` (`scripts/discover.py`, `scripts/get_local_keys.py`). Keys never go into this repo.

## How it works

| Version | Status | Switch | Frame check |
| --- | --- | --- | --- |
| 3.3 | cmd 10, AES(key, JSON) | cmd 7, `"3.3"` + 12 zeros + AES(key, JSON) | CRC32 |
| 3.4 | cmd 16, AES(session, `{}`) | cmd 13, AES(session, `"3.4"` + 12 zeros + JSON) | HMAC-SHA256 |

For 3.4 the robot and the device first agree on a session key: our nonce (cmd 3), theirs with an HMAC of ours (cmd 4), an HMAC of theirs (cmd 5). The session key is AES-ECB(local key, our nonce XOR theirs). AES is ECB with PKCS#7 padding. The messages are the ones tinytuya sends; the host tests check our bytes against tinytuya's.

- **One connection per call.** Each call opens TCP, does one thing and closes. Some devices take only one connection at a time (our power strip refuses everyone while the phone app holds it), so we never keep one open.
- **One call at a time.** A mutex: the screen, the voice and the web server may all switch something.
- **Memory.** Each call takes one 3 KB block, so the robot puts it in PSRAM.
- **Portable.** `tr_tuya.c` is plain C with BSD sockets. The crypto is in `tr_tuya_crypto.h`: PSA on the robot (`tr_tuya_esp.c`, AES on the chip's hardware), CommonCrypto or OpenSSL in the host tests.

## Errors

| Code | Meaning |
| --- | --- |
| `TR_TUYA_ERR_CONNECT` | No TCP connection: the device is off, the IP changed, or it is busy with another client (the phone app) |
| `TR_TUYA_ERR_IO` | The connection broke, or no answer within 3 s |
| `TR_TUYA_ERR_FRAME` | A bad answer: usually the wrong key or the wrong version |
| `TR_TUYA_ERR_SESSION` | 3.4 only: the device refused our session key (wrong key) |

## Tests

```bash
scripts/test-host.sh
```

`tests/host/test_tuya.c` checks frames, the session key, decryption and the data point parser against bytes made by tinytuya with a made-up key. `tests/host/tuya_cli.c` runs the same code against a real device from a computer:

```bash
cc -std=c11 -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE -Icomponents/tr_tuya/include \
   tests/host/tuya_cli.c components/tr_tuya/tr_tuya.c tests/host/tuya_crypto_host.c -o .tools/tuya_cli
TUYA_KEY=<32 hex> .tools/tuya_cli 192.168.1.36 <id> 34
```

Checked on 2026-11-02 from a Mac: status and switch on two SP112 plugs (3.4) and a power strip (3.3).

Not supported: version 3.5 (AES-GCM, prefix 0x6699), 3.1, and "device22" devices.
