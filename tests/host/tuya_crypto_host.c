// tr_tuya's crypto on a computer, for the host tests: CommonCrypto on macOS,
// OpenSSL elsewhere. The robot uses PSA (components/tr_tuya/tr_tuya_esp.c).
#include <stdlib.h>
#include "../../components/tr_tuya/tr_tuya_crypto.h"

#ifdef __APPLE__
#include <CommonCrypto/CommonCryptor.h>
#include <CommonCrypto/CommonHMAC.h>

int tr_tuya_aes_ecb(const uint8_t key[16], bool encrypt, const uint8_t *in, uint8_t *out,
                    size_t len)
{
    size_t moved = 0;
    const CCCryptorStatus s = CCCrypt(encrypt ? kCCEncrypt : kCCDecrypt, kCCAlgorithmAES128,
                                      kCCOptionECBMode, key, 16, NULL, in, len, out, len, &moved);
    return s == kCCSuccess && moved == len ? 0 : -1;
}

int tr_tuya_hmac(const uint8_t key[16], const uint8_t *data, size_t len, uint8_t out[32])
{
    CCHmac(kCCHmacAlgSHA256, key, 16, data, len, out);
    return 0;
}

void tr_tuya_random(uint8_t *out, size_t len)
{
    arc4random_buf(out, len);
}
#else
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/rand.h>

int tr_tuya_aes_ecb(const uint8_t key[16], bool encrypt, const uint8_t *in, uint8_t *out,
                    size_t len)
{
    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    int n = 0, m = 0;
    const int ok = ctx && EVP_CipherInit_ex(ctx, EVP_aes_128_ecb(), NULL, key, NULL, encrypt) &&
                   EVP_CIPHER_CTX_set_padding(ctx, 0) &&
                   EVP_CipherUpdate(ctx, out, &n, in, (int)len) &&
                   EVP_CipherFinal_ex(ctx, out + n, &m);
    EVP_CIPHER_CTX_free(ctx);
    return ok && (size_t)(n + m) == len ? 0 : -1;
}

int tr_tuya_hmac(const uint8_t key[16], const uint8_t *data, size_t len, uint8_t out[32])
{
    unsigned n = 32;
    return HMAC(EVP_sha256(), key, 16, data, len, out, &n) ? 0 : -1;
}

void tr_tuya_random(uint8_t *out, size_t len)
{
    RAND_bytes(out, (int)len);
}
#endif

void tr_tuya_lock(void)
{
}
void tr_tuya_unlock(void)
{
}
