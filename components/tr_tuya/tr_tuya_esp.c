// tr_tuya's crypto and lock on the robot: PSA (mbedTLS 4 in ESP-IDF 6), which
// does AES with the chip's AES hardware, and a FreeRTOS mutex.
#include <stdlib.h>
#include <string.h>
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "psa/crypto.h"
#include "tr_tuya_crypto.h"

static psa_key_id_t import(const uint8_t key[16], psa_key_type_t type, psa_key_usage_t usage,
                           psa_algorithm_t alg)
{
    static bool ready;
    if (!ready) {
        if (psa_crypto_init() != PSA_SUCCESS) {
            return 0;
        }
        ready = true;
    }
    psa_key_attributes_t a = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&a, type);
    psa_set_key_bits(&a, 128);
    psa_set_key_usage_flags(&a, usage);
    psa_set_key_algorithm(&a, alg);
    psa_key_id_t id = 0;
    return psa_import_key(&a, key, 16, &id) == PSA_SUCCESS ? id : 0;
}

int tr_tuya_aes_ecb(const uint8_t key[16], bool encrypt, const uint8_t *in, uint8_t *out,
                    size_t len)
{
    const psa_key_id_t id =
        import(key, PSA_KEY_TYPE_AES, encrypt ? PSA_KEY_USAGE_ENCRYPT : PSA_KEY_USAGE_DECRYPT,
               PSA_ALG_ECB_NO_PADDING);
    if (!id) {
        return -1;
    }
    // PSA does not promise that in and out may be the same buffer.
    uint8_t *copy = malloc(len);
    psa_status_t s = PSA_ERROR_INSUFFICIENT_MEMORY;
    size_t moved = 0;
    if (copy) {
        memcpy(copy, in, len);
        s = encrypt ? psa_cipher_encrypt(id, PSA_ALG_ECB_NO_PADDING, copy, len, out, len, &moved)
                    : psa_cipher_decrypt(id, PSA_ALG_ECB_NO_PADDING, copy, len, out, len, &moved);
        free(copy);
    }
    psa_destroy_key(id);
    return s == PSA_SUCCESS && moved == len ? 0 : -1;
}

int tr_tuya_hmac(const uint8_t key[16], const uint8_t *data, size_t len, uint8_t out[32])
{
    const psa_algorithm_t alg = PSA_ALG_HMAC(PSA_ALG_SHA_256);
    const psa_key_id_t id = import(key, PSA_KEY_TYPE_HMAC, PSA_KEY_USAGE_SIGN_MESSAGE, alg);
    if (!id) {
        return -1;
    }
    size_t n = 0;
    const psa_status_t s = psa_mac_compute(id, alg, data, len, out, 32, &n);
    psa_destroy_key(id);
    return s == PSA_SUCCESS && n == 32 ? 0 : -1;
}

void tr_tuya_random(uint8_t *out, size_t len)
{
    esp_fill_random(out, len);
}

static SemaphoreHandle_t lock;

void tr_tuya_lock(void)
{
    if (!lock) {
        // The first call comes before any other task can call: the example
        // reads the devices once at start.
        lock = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(lock, portMAX_DELAY);
}

void tr_tuya_unlock(void)
{
    xSemaphoreGive(lock);
}
