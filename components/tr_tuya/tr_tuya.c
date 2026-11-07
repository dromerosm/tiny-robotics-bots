// tr_tuya: Tuya's local protocol, 3.3 and 3.4. See include/tr_tuya.h.
//
// The messages, as tinytuya (Python) sends them:
//
//   3.3 status   cmd 10, AES(key, {"gwId","devId","uid","t"})
//   3.3 switch   cmd 7,  "3.3" + 12 zero bytes + AES(key, {"devId","uid","t","dps"})
//   3.4 session  cmd 3 (our nonce) -> cmd 4 (theirs + HMAC of ours) -> cmd 5
//   3.4 status   cmd 16, AES(session, {})
//   3.4 switch   cmd 13, AES(session, "3.4" + 12 zero bytes + {"protocol":5,"t","data":{"dps"}})
//
// 3.3 frames end in a CRC32, 3.4 frames in an HMAC-SHA256 (with the local key
// while the session is agreed, then with the session key).
#include "tr_tuya.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include "tr_tuya_crypto.h"

#define PORT         6668
#define PREFIX       0x000055AAu
#define SUFFIX       0x0000AA55u
#define CONNECT_MS   2000
#define IO_MS        3000
#define TX_MAX       512
#define RX_MAX       1536
#define PLAIN_MAX    1024
#define CMD_SESS_1   3
#define CMD_SESS_2   4
#define CMD_SESS_3   5
#define CMD_CONTROL  7
#define CMD_STATUS   8
#define CMD_QUERY    10
#define CMD_CONTROL4 13
#define CMD_QUERY4   16

// --- Frames ----------------------------------------------------------------

static uint32_t crc32(const uint8_t *p, size_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    while (len--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) {
            c = c & 1 ? (c >> 1) ^ 0xEDB88320u : c >> 1;
        }
    }
    return ~c;
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

size_t tr_tuya_frame(uint8_t *out, size_t cap, uint32_t seq, uint32_t cmd, const uint8_t *payload,
                     size_t len, const uint8_t *hmac_key)
{
    const size_t check = hmac_key ? 32 : 4;
    const size_t total = 16 + len + check + 4;
    if (total > cap) {
        return 0;
    }
    put32(out, PREFIX);
    put32(out + 4, seq);
    put32(out + 8, cmd);
    put32(out + 12, (uint32_t)(len + check + 4));
    memcpy(out + 16, payload, len);
    if (hmac_key) {
        if (tr_tuya_hmac(hmac_key, out, 16 + len, out + 16 + len) != 0) {
            return 0;
        }
    } else {
        put32(out + 16 + len, crc32(out, 16 + len));
    }
    put32(out + total - 4, SUFFIX);
    return total;
}

int tr_tuya_unframe(const uint8_t *in, size_t len, const uint8_t *hmac_key, uint32_t *cmd,
                    const uint8_t **payload, size_t *plen)
{
    if (len < 16) {
        return 0;
    }
    if (get32(in) != PREFIX) {
        return TR_TUYA_ERR_FRAME;
    }
    const size_t body = get32(in + 12), total = 16 + body;
    const size_t end = hmac_key ? 36 : 8;
    if (body < 4 + end || body > RX_MAX) {
        return TR_TUYA_ERR_FRAME;
    }
    if (len < total) {
        return 0;
    }
    const size_t checked = total - end;
    if (hmac_key) {
        uint8_t mac[32];
        if (tr_tuya_hmac(hmac_key, in, checked, mac) != 0 || memcmp(mac, in + checked, 32) != 0) {
            return TR_TUYA_ERR_FRAME;
        }
    } else if (crc32(in, checked) != get32(in + checked)) {
        return TR_TUYA_ERR_FRAME;
    }
    *cmd = get32(in + 8);
    *payload = in + 20; // after the return code
    *plen = checked - 20;
    return (int)total;
}

// --- AES and the session key -------------------------------------------------

int tr_tuya_encrypt(const uint8_t key[16], const uint8_t *in, size_t len, uint8_t *out)
{
    const size_t pad = 16 - len % 16;
    memmove(out, in, len);
    memset(out + len, (int)pad, pad);
    return tr_tuya_aes_ecb(key, true, out, out, len + pad) == 0 ? (int)(len + pad)
                                                                : TR_TUYA_ERR_FRAME;
}

int tr_tuya_decrypt(const uint8_t key[16], const uint8_t *in, size_t len, uint8_t *out)
{
    if (len == 0 || len % 16 != 0 || tr_tuya_aes_ecb(key, false, in, out, len) != 0) {
        return TR_TUYA_ERR_FRAME;
    }
    const uint8_t pad = out[len - 1];
    if (pad == 0 || pad > 16) {
        return TR_TUYA_ERR_FRAME;
    }
    return (int)(len - pad);
}

int tr_tuya_session_key(const uint8_t key[16], const uint8_t local_nonce[16],
                        const uint8_t remote_nonce[16], uint8_t session[16])
{
    uint8_t x[16];
    for (int i = 0; i < 16; i++) {
        x[i] = local_nonce[i] ^ remote_nonce[i];
    }
    return tr_tuya_aes_ecb(key, true, x, session, 16);
}

// --- JSON ------------------------------------------------------------------

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') {
        p++;
    }
    return p;
}

// Skip one JSON value (string, number, literal, object or array).
static const char *skip_value(const char *p)
{
    p = skip_ws(p);
    if (*p == '"') {
        for (p++; *p && *p != '"'; p++) {
            if (*p == '\\' && p[1]) {
                p++;
            }
        }
        return *p ? p + 1 : p;
    }
    if (*p == '{' || *p == '[') {
        int depth = 0;
        for (; *p; p++) {
            if (*p == '"') {
                p = skip_value(p) - 1;
            } else if (*p == '{' || *p == '[') {
                depth++;
            } else if ((*p == '}' || *p == ']') && --depth == 0) {
                return p + 1;
            }
        }
        return p;
    }
    while (*p && *p != ',' && *p != '}' && *p != ']') {
        p++;
    }
    return p;
}

int tr_tuya_parse_dps(const char *json, tr_tuya_dp_t *dps, int max)
{
    const char *p = strstr(json, "\"dps\"");
    if (!p) {
        return 0;
    }
    p = skip_ws(p + 5);
    if (*p++ != ':') {
        return 0;
    }
    p = skip_ws(p);
    if (*p++ != '{') {
        return 0;
    }
    int n = 0;
    for (;;) {
        p = skip_ws(p);
        if (*p != '"') {
            return n;
        }
        const int dp = atoi(p + 1);
        p = strchr(p + 1, '"');
        if (!p) {
            return n;
        }
        p = skip_ws(p + 1);
        if (*p++ != ':') {
            return n;
        }
        p = skip_ws(p);
        int value = 0;
        bool keep = true;
        if (strncmp(p, "true", 4) == 0) {
            value = 1;
        } else if (strncmp(p, "false", 5) == 0) {
            value = 0;
        } else if (*p == '-' || (*p >= '0' && *p <= '9')) {
            value = atoi(p);
        } else {
            keep = false; // text or something else: left out
        }
        if (keep && n < max) {
            dps[n].dp = dp;
            dps[n].value = value;
            n++;
        }
        p = skip_ws(skip_value(p));
        if (*p != ',') {
            return n;
        }
        p++;
    }
}

int tr_tuya_value(const tr_tuya_dp_t *dps, int count, int dp)
{
    for (int i = 0; i < count; i++) {
        if (dps[i].dp == dp) {
            return dps[i].value;
        }
    }
    return -1;
}

bool tr_tuya_key_from_hex(const char *hex, uint8_t key[16])
{
    if (!hex || strlen(hex) != 32) {
        return false;
    }
    for (int i = 0; i < 16; i++) {
        unsigned v;
        if (sscanf(hex + 2 * i, "%2x", &v) != 1) {
            return false;
        }
        key[i] = (uint8_t)v;
    }
    return true;
}

const char *tr_tuya_err_text(int err)
{
    switch (err) {
        case TR_TUYA_ERR_CONNECT:
            return "no connection (off, busy, or wrong IP)";
        case TR_TUYA_ERR_IO:
            return "connection lost or no answer";
        case TR_TUYA_ERR_FRAME:
            return "bad answer (wrong key or version)";
        case TR_TUYA_ERR_SESSION:
            return "session key refused (wrong key)";
        case TR_TUYA_ERR_ARG:
            return "bad argument";
        default:
            return err >= 0 ? "ok" : "error";
    }
}

// --- One connection ----------------------------------------------------------

typedef struct {
    const tr_tuya_dev_t *dev;
    int sock;
    uint32_t seq;
    uint8_t session[16]; // 3.4: the session key
    uint8_t *tx;         // TX_MAX
    uint8_t *rx;         // RX_MAX
    size_t rx_len;
    size_t rx_used; // bytes of the last frame, dropped on the next read
    char *plain;    // PLAIN_MAX + 1
} conn_t;

static int tcp_open(const char *ip)
{
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons(PORT)};
    if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1) {
        return TR_TUYA_ERR_ARG;
    }
    const int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) {
        return TR_TUYA_ERR_CONNECT;
    }
    // Connect with a time limit: non-blocking, then wait until it is writable.
    const int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
    int r = connect(s, (struct sockaddr *)&addr, sizeof(addr));
    if (r < 0 && errno == EINPROGRESS) {
        fd_set w;
        FD_ZERO(&w);
        FD_SET(s, &w);
        struct timeval tv = {.tv_sec = CONNECT_MS / 1000, .tv_usec = CONNECT_MS % 1000 * 1000};
        int err = 0;
        socklen_t elen = sizeof(err);
        r = select(s + 1, NULL, &w, NULL, &tv) == 1 &&
                    getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &elen) == 0 && err == 0
                ? 0
                : -1;
    }
    if (r < 0) {
        close(s);
        return TR_TUYA_ERR_CONNECT;
    }
    fcntl(s, F_SETFL, flags);
    struct timeval io = {.tv_sec = IO_MS / 1000, .tv_usec = IO_MS % 1000 * 1000};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &io, sizeof(io));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &io, sizeof(io));
    const int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return s;
}

static int send_frame(conn_t *c, uint32_t cmd, const uint8_t *payload, size_t len,
                      const uint8_t *hmac_key)
{
    const size_t n = tr_tuya_frame(c->tx, TX_MAX, ++c->seq, cmd, payload, len, hmac_key);
    if (n == 0) {
        return TR_TUYA_ERR_ARG;
    }
    for (size_t sent = 0; sent < n;) {
        const ssize_t k = send(c->sock, c->tx + sent, n - sent, 0);
        if (k <= 0) {
            return TR_TUYA_ERR_IO;
        }
        sent += (size_t)k;
    }
    return 0;
}

// Wait for the next frame. On success *payload points into c->rx and stays
// good until the next call.
static int recv_frame(conn_t *c, const uint8_t *hmac_key, uint32_t *cmd, const uint8_t **payload,
                      size_t *plen)
{
    if (c->rx_used) {
        memmove(c->rx, c->rx + c->rx_used, c->rx_len - c->rx_used);
        c->rx_len -= c->rx_used;
        c->rx_used = 0;
    }
    for (;;) {
        const int r = tr_tuya_unframe(c->rx, c->rx_len, hmac_key, cmd, payload, plen);
        if (r > 0) {
            c->rx_used = (size_t)r;
            return 0;
        }
        if (r < 0) {
            return r;
        }
        if (c->rx_len == RX_MAX) {
            return TR_TUYA_ERR_FRAME;
        }
        const ssize_t k = recv(c->sock, c->rx + c->rx_len, RX_MAX - c->rx_len, 0);
        if (k <= 0) {
            return TR_TUYA_ERR_IO;
        }
        c->rx_len += (size_t)k;
    }
}

// The JSON in an answer, as text in c->plain (empty for a plain ack).
static int open_payload(conn_t *c, const uint8_t *p, size_t len)
{
    int n = 0;
    if (c->dev->version >= 34) {
        if (len > 0) {
            if (len > PLAIN_MAX) {
                return TR_TUYA_ERR_FRAME;
            }
            n = tr_tuya_decrypt(c->session, p, len, (uint8_t *)c->plain);
            if (n < 0) {
                return n;
            }
            if (n >= 15 && memcmp(c->plain, "3.4", 3) == 0) {
                memmove(c->plain, c->plain + 15, (size_t)(n - 15));
                n -= 15;
            }
        }
    } else {
        if (len >= 15 && memcmp(p, "3.3", 3) == 0) {
            p += 15;
            len -= 15;
        }
        if (len > 0) {
            if (len > PLAIN_MAX) {
                return TR_TUYA_ERR_FRAME;
            }
            n = tr_tuya_decrypt(c->dev->key, p, len, (uint8_t *)c->plain);
            if (n < 0) {
                return n;
            }
        }
    }
    c->plain[n] = 0;
    return n;
}

static int negotiate(conn_t *c)
{
    const uint8_t *key = c->dev->key;
    uint8_t nonce[16], buf[64];
    tr_tuya_random(nonce, sizeof(nonce));
    int n = tr_tuya_encrypt(key, nonce, sizeof(nonce), buf);
    if (n < 0 || send_frame(c, CMD_SESS_1, buf, (size_t)n, key) != 0) {
        return TR_TUYA_ERR_IO;
    }
    uint32_t cmd;
    const uint8_t *p;
    size_t len;
    int r = recv_frame(c, key, &cmd, &p, &len);
    if (r != 0) {
        return r == TR_TUYA_ERR_FRAME ? TR_TUYA_ERR_SESSION : r;
    }
    if (cmd != CMD_SESS_2 || len > sizeof(buf) || (n = tr_tuya_decrypt(key, p, len, buf)) < 48) {
        return TR_TUYA_ERR_SESSION;
    }
    uint8_t remote[16], mac[32];
    memcpy(remote, buf, 16);
    if (tr_tuya_hmac(key, nonce, 16, mac) != 0 || memcmp(mac, buf + 16, 32) != 0) {
        return TR_TUYA_ERR_SESSION;
    }
    if (tr_tuya_hmac(key, remote, 16, mac) != 0 || (n = tr_tuya_encrypt(key, mac, 32, buf)) < 0 ||
        send_frame(c, CMD_SESS_3, buf, (size_t)n, key) != 0) {
        return TR_TUYA_ERR_IO;
    }
    return tr_tuya_session_key(key, nonce, remote, c->session) == 0 ? 0 : TR_TUYA_ERR_SESSION;
}

static int conn_open(conn_t *c, const tr_tuya_dev_t *dev)
{
    memset(c, 0, sizeof(*c));
    c->dev = dev;
    c->sock = -1;
    if (!dev || !dev->id || !dev->ip || (dev->version != 33 && dev->version != 34)) {
        return TR_TUYA_ERR_ARG;
    }
    // One block, above 2 KB, so the robot puts it in PSRAM.
    uint8_t *mem = malloc(TX_MAX + RX_MAX + PLAIN_MAX + 1);
    if (!mem) {
        return TR_TUYA_ERR_IO;
    }
    c->tx = mem;
    c->rx = mem + TX_MAX;
    c->plain = (char *)(mem + TX_MAX + RX_MAX);
    c->sock = tcp_open(dev->ip);
    if (c->sock < 0) {
        return c->sock;
    }
    return dev->version >= 34 ? negotiate(c) : 0;
}

static void conn_close(conn_t *c)
{
    if (c->sock >= 0) {
        close(c->sock);
    }
    free(c->tx);
}

// Encrypt the JSON for this device and send it. header adds "3.x" + 12 zeros.
static int send_json(conn_t *c, uint32_t cmd, const char *json, bool header)
{
    uint8_t buf[TX_MAX - 64];
    const size_t len = strlen(json);
    if (len + 15 + 16 > sizeof(buf)) {
        return TR_TUYA_ERR_ARG;
    }
    int n;
    if (c->dev->version >= 34) {
        uint8_t plain[sizeof(buf)];
        size_t m = 0;
        if (header) {
            memcpy(plain, "3.4", 3);
            memset(plain + 3, 0, 12);
            m = 15;
        }
        memcpy(plain + m, json, len);
        n = tr_tuya_encrypt(c->session, plain, m + len, buf);
        return n < 0 ? n : send_frame(c, cmd, buf, (size_t)n, c->session);
    }
    size_t m = 0;
    if (header) {
        memcpy(buf, "3.3", 3);
        memset(buf + 3, 0, 12);
        m = 15;
    }
    n = tr_tuya_encrypt(c->dev->key, (const uint8_t *)json, len, buf + m);
    return n < 0 ? n : send_frame(c, cmd, buf, m + (size_t)n, NULL);
}

static const uint8_t *frame_key(const conn_t *c)
{
    return c->dev->version >= 34 ? c->session : NULL;
}

int tr_tuya_status(const tr_tuya_dev_t *dev, tr_tuya_dp_t *dps, int max)
{
    tr_tuya_lock();
    conn_t c;
    int r = conn_open(&c, dev);
    if (r == 0) {
        char json[160];
        const long t = (long)time(NULL);
        if (dev->version >= 34) {
            r = send_json(&c, CMD_QUERY4, "{}", false);
        } else {
            snprintf(json, sizeof(json),
                     "{\"gwId\":\"%s\",\"devId\":\"%s\",\"uid\":\"%s\",\"t\":\"%ld\"}", dev->id,
                     dev->id, dev->id, t);
            r = send_json(&c, CMD_QUERY, json, false);
        }
    }
    // The answer is the frame with our command. Before it, a device may send
    // an empty ack, a heartbeat, or a status push (cmd 8) with only some data
    // points (the power strip sends its voltage): those are merged in, but
    // they are not the answer.
    const uint32_t query = dev->version >= 34 ? CMD_QUERY4 : CMD_QUERY;
    int count = 0;
    bool answered = false;
    for (int tries = 0; r == 0 && !answered && tries < 6; tries++) {
        uint32_t cmd;
        const uint8_t *p;
        size_t len;
        r = recv_frame(&c, frame_key(&c), &cmd, &p, &len);
        if (r == 0 && (r = open_payload(&c, p, len)) >= 0) {
            r = 0;
            if (strstr(c.plain, "\"dps\"")) {
                tr_tuya_dp_t got[32];
                const int n = tr_tuya_parse_dps(c.plain, got, 32);
                for (int i = 0; i < n; i++) {
                    int k = 0;
                    while (k < count && dps[k].dp != got[i].dp) {
                        k++;
                    }
                    if (k < max) {
                        dps[k] = got[i];
                        count += k == count;
                    }
                }
                answered = cmd == query;
            }
        }
    }
    if (r == 0) {
        r = answered ? count : TR_TUYA_ERR_IO;
    }
    conn_close(&c);
    tr_tuya_unlock();
    return r;
}

int tr_tuya_set(const tr_tuya_dev_t *dev, int dp, bool on)
{
    tr_tuya_lock();
    conn_t c;
    int r = conn_open(&c, dev);
    uint32_t sent = 0;
    if (r == 0) {
        char json[200];
        const long t = (long)time(NULL);
        if (dev->version >= 34) {
            snprintf(json, sizeof(json),
                     "{\"protocol\":5,\"t\":%ld,\"data\":{\"dps\":{\"%d\":%s}}}", t, dp,
                     on ? "true" : "false");
            sent = CMD_CONTROL4;
        } else {
            snprintf(json, sizeof(json),
                     "{\"devId\":\"%s\",\"uid\":\"%s\",\"t\":\"%ld\",\"dps\":{\"%d\":%s}}", dev->id,
                     dev->id, t, dp, on ? "true" : "false");
            sent = CMD_CONTROL;
        }
        r = send_json(&c, sent, json, true);
    }
    // The device answers with the same command (an ack), then a status.
    for (int tries = 0; r == 0 && tries < 4; tries++) {
        uint32_t cmd;
        const uint8_t *p;
        size_t len;
        r = recv_frame(&c, frame_key(&c), &cmd, &p, &len);
        if (r == 0 && (cmd == sent || cmd == CMD_STATUS)) {
            break;
        }
        if (r == 0 && tries == 3) {
            r = TR_TUYA_ERR_FRAME;
        }
    }
    conn_close(&c);
    tr_tuya_unlock();
    return r;
}
