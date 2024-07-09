#ifndef CRYPTO_H
#define CRYPTO_H

#include <stdint.h>
#include <stddef.h>

/**
 * @file crypto.h
 * @author Bykowski Olaf
 *
 * @brief This file contains declarations for the crypto module.
 */

typedef enum
{
    CRYPTO_OK,
    CRYPTO_AES_INIT_ERROR,
    CRYPTO_AES_SETKEY_ERROR,
    CRYPTO_AES_CRYPT_ERROR,
} crypto_status_t;

typedef enum {
    CRYPTO_OPERATION_ENCRYPT,
    CRYPTO_OPERATION_DECRYPT,
} crypto_operation_t;

crypto_status_t crypto_crypt(uint8_t *input, size_t size, uint8_t *output, uint8_t *iv, crypto_operation_t operation);

#endif // CRYPTO_H
