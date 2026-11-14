// Host tests for tr_tuya's protocol pieces. The expected bytes come from
// tinytuya (Python) with the made-up key "0123456789abcdef".
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tr_tuya.h"

static int failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
            failures++;                                                                            \
        }                                                                                          \
    } while (0)

static const uint8_t KEY[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

static size_t from_hex(const char *hex, uint8_t *out)
{
    size_t n = 0;
    for (; hex[2 * n]; n++) {
        unsigned v;
        sscanf(hex + 2 * n, "%2x", &v);
        out[n] = (uint8_t)v;
    }
    return n;
}

// A 3.3 status request: AES of the JSON, framed with a CRC32.
static void test_frame_33(void)
{
    const char *json = "{\"gwId\":\"abc\",\"devId\":\"abc\",\"uid\":\"abc\",\"t\":\"1700000000\"}";
    uint8_t enc[128], frame[256], want[256];
    const int n = tr_tuya_encrypt(KEY, (const uint8_t *)json, strlen(json), enc);
    CHECK(n == 64);
    const size_t len = tr_tuya_frame(frame, sizeof(frame), 1, 10, enc, (size_t)n, NULL);
    const size_t wlen = from_hex(
        "000055aa000000010000000a000000484fccec16027646a67de7d69a81745b28f453edabddc7e4a952cbe"
        "bbac1e4aed6552ae08abc57c95ce6eea621e55242e765ac5e1d1e3c3c395817ca95740225b43360e958"
        "0000aa55",
        want);
    CHECK(len == wlen && memcmp(frame, want, wlen) == 0);
}

// A 3.4 switch: "3.4" + 12 zeros + JSON, AES, framed with HMAC-SHA256.
static void test_frame_34(void)
{
    const char *json = "{\"protocol\":5,\"t\":1700000000,\"data\":{\"dps\":{\"1\":true}}}";
    uint8_t plain[128] = "3.4", enc[160], frame[256], want[256];
    memset(plain + 3, 0, 12);
    memcpy(plain + 15, json, strlen(json));
    const int n = tr_tuya_encrypt(KEY, plain, 15 + strlen(json), enc);
    const size_t len = tr_tuya_frame(frame, sizeof(frame), 7, 13, enc, (size_t)n, KEY);
    const size_t wlen = from_hex("000055aa000000070000000d000000744490b05d74be9368c24a038cbaeded8e8"
                                 "9ea503e4be494dfe0c010bc6c463c75723ef6ac"
                                 "a177a39b03b3e4fb00012c19edd249953145abb3cfa65e18d98cc3bde3d0951b8"
                                 "ab2e031cb512d241494e9990fedfaa57b8dd42b8d"
                                 "c8dbc42d7dbf6f4fc9df2b4f38c4449264fac7e61afa590000aa55",
                                 want);
    CHECK(len == wlen && memcmp(frame, want, wlen) == 0);
}

static void test_session_key(void)
{
    uint8_t session[16], want[16];
    CHECK(tr_tuya_session_key(KEY, (const uint8_t *)"ABCDEFGHIJKLMNOP",
                              (const uint8_t *)"ponmlkjihgfedcba", session) == 0);
    from_hex("c7012a76db1a00da4b0f7ff90ae327c7", want);
    CHECK(memcmp(session, want, 16) == 0);
}

// A device's 3.3 answer: return code, AES of the JSON, CRC32.
static void test_answer_33(void)
{
    uint8_t in[256];
    const size_t n = from_hex(
        "000055aa000000020000000a0000004c0000000023d681a3c46b76acf440f1db5dc5f259fc8ea47f00e5"
        "c661bae58ec2d66081c979486ec49871c8890f3c7e3e791b585545e08ac39d9faf27672fd656838952"
        "947a5cdace0000aa55",
        in);
    uint32_t cmd = 0;
    const uint8_t *p;
    size_t plen;
    CHECK(tr_tuya_unframe(in, n - 1, NULL, &cmd, &p, &plen) == 0); // one byte short: wait
    CHECK(tr_tuya_unframe(in, n, NULL, &cmd, &p, &plen) == (int)n);
    CHECK(cmd == 10 && plen == 64);
    char json[128];
    const int m = tr_tuya_decrypt(KEY, p, plen, (uint8_t *)json);
    CHECK(m > 0);
    json[m > 0 ? m : 0] = 0;
    tr_tuya_dp_t dps[8];
    const int count = tr_tuya_parse_dps(json, dps, 8);
    CHECK(count == 3); // "38":"2" is text: left out
    CHECK(tr_tuya_value(dps, count, 1) == 1);
    CHECK(tr_tuya_value(dps, count, 7) == 0);
    CHECK(tr_tuya_value(dps, count, 19) == 308);
    CHECK(tr_tuya_value(dps, count, 38) == -1);
    in[30] ^= 1; // a broken byte: the CRC check fails
    CHECK(tr_tuya_unframe(in, n, NULL, &cmd, &p, &plen) == TR_TUYA_ERR_FRAME);
}

static void test_parse(void)
{
    tr_tuya_dp_t dps[8];
    const char *pushed = "{\"protocol\":4,\"t\":1,\"data\":{\"dps\":{\"1\": true , \"20\":2162}}}";
    int n = tr_tuya_parse_dps(pushed, dps, 8);
    CHECK(n == 2 && tr_tuya_value(dps, n, 20) == 2162);
    const char *nested = "{\"dps\":{\"5\":{\"a\":\"}\"},\"6\":false}}";
    n = tr_tuya_parse_dps(nested, dps, 8);
    CHECK(n == 1 && tr_tuya_value(dps, n, 6) == 0);
    CHECK(tr_tuya_parse_dps("{\"devId\":\"x\"}", dps, 8) == 0);
    uint8_t key[16];
    CHECK(tr_tuya_key_from_hex("30313233343536373839616263646566", key) &&
          memcmp(key, KEY, 16) == 0);
    CHECK(!tr_tuya_key_from_hex("3031", key));
}

// A broadcast as a device sends it on UDP 6667: a frame with a return code,
// then the JSON under the key every device shares.
static void test_broadcast(void)
{
    static const uint8_t udp_key[16] = {0x6c, 0x1e, 0xc8, 0xe2, 0xbb, 0x9b, 0xb5, 0x9a,
                                        0xb5, 0x0b, 0x0d, 0xaf, 0x64, 0x9b, 0x41, 0x0a};
    const char *json = "{\"ip\":\"192.168.1.77\",\"gwId\":\"bf00112233445566778899\","
                       "\"active\":2,\"encrypt\":true,\"version\":\"3.4\"}";
    uint8_t body[256] = {0}, frame[300];
    const int n = tr_tuya_encrypt(udp_key, (const uint8_t *)json, strlen(json), body + 4);
    size_t len = tr_tuya_frame(frame, sizeof(frame), 0, 19, body, (size_t)n + 4, NULL);
    char id[24], ip[16];
    CHECK(tr_tuya_broadcast(frame, len, id, sizeof(id), ip, sizeof(ip)) == 0);
    CHECK(strcmp(id, "bf00112233445566778899") == 0 && strcmp(ip, "192.168.1.77") == 0);
    // Port 6666 sends the same without AES.
    memcpy(body + 4, json, strlen(json));
    len = tr_tuya_frame(frame, sizeof(frame), 0, 19, body, strlen(json) + 4, NULL);
    CHECK(tr_tuya_broadcast(frame, len, id, sizeof(id), ip, sizeof(ip)) == 0);
    CHECK(strcmp(ip, "192.168.1.77") == 0);
    // An id that does not fit, and bytes that are not a frame.
    CHECK(tr_tuya_broadcast(frame, len, id, 8, ip, sizeof(ip)) == TR_TUYA_ERR_FRAME);
    CHECK(tr_tuya_broadcast((const uint8_t *)json, strlen(json), id, sizeof(id), ip, sizeof(ip)) ==
          TR_TUYA_ERR_FRAME);
}

int main(void)
{
    test_frame_33();
    test_frame_34();
    test_session_key();
    test_answer_33();
    test_parse();
    test_broadcast();
    printf("%s: %s\n", __FILE__, failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
