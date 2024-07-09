#include "crypto.h"
#include <stddef.h>
#include "esp_log.h"
#include "mbedtls/cipher.h"

const static char *CRYPTO_TAG = "CRYPTO";

static uint8_t key[16] = CONFIG_CRYPTO_KEY;

crypto_status_t crypto_crypt(uint8_t *input, size_t size, uint8_t *output, uint8_t *iv, crypto_operation_t operation)
{
    int ret;
    const mbedtls_cipher_info_t *cipher_info;
    mbedtls_cipher_context_t cipher_ctx;

    // Initialize the cipher context
    mbedtls_cipher_init(&cipher_ctx);

    // Get the cipher info for AES-128-CTR
    cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CTR);
    if (cipher_info == NULL) {
        ESP_LOGE(CRYPTO_TAG, "Cipher info not found");
        mbedtls_cipher_free(&cipher_ctx);
        return CRYPTO_AES_INIT_ERROR;
    }

    // Setup the cipher context
    ret = mbedtls_cipher_setup(&cipher_ctx, cipher_info);
    if (ret != 0) {
        ESP_LOGE(CRYPTO_TAG, "mbedtls_cipher_setup failed: %d", ret);
        mbedtls_cipher_free(&cipher_ctx);
        return CRYPTO_AES_INIT_ERROR;
    }

    // Set the key for the cipher context
    ret = mbedtls_cipher_setkey(&cipher_ctx, key, 128, (operation == CRYPTO_OPERATION_ENCRYPT) ? MBEDTLS_ENCRYPT : MBEDTLS_DECRYPT);
    if (ret != 0) {
        ESP_LOGE(CRYPTO_TAG, "mbedtls_cipher_setkey failed: %d", ret);
        mbedtls_cipher_free(&cipher_ctx);
        return CRYPTO_AES_SETKEY_ERROR;
    }

    ret = mbedtls_cipher_crypt(&cipher_ctx, iv, 16, input, size, output, &size);
    if (ret != 0) {
        ESP_LOGE(CRYPTO_TAG, "mbedtls_cipher_crypt failed: %d", ret);
        mbedtls_cipher_free(&cipher_ctx);
        return CRYPTO_AES_CRYPT_ERROR;
    }

    // Free the cipher context
    mbedtls_cipher_free(&cipher_ctx);

    return CRYPTO_OK;
}