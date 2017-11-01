/*
 * crypto.c
 *
 *  Created on: Oct 4, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#include "crypto.h"
#include "Systick.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include <limits.h>
#include "nrf_sdh_soc.h"
#include "internal_memory_organization.h"
#include "internal_flash.h"
#include <string.h>
#include <malloc.h>

uint8_t currentInitialisingVector[CRYPTO_KEY_SIZE];
uint8_t mainEncryptionKey[CRYPTO_KEY_SIZE];

extern void AES_128_keyschedule(const uint8_t *, uint8_t *);
extern void AES_128_keyschedule_dec(const uint8_t *, uint8_t *);
extern void AES_128_encrypt(const uint8_t *, const uint8_t *, uint8_t *);
extern void AES_128_decrypt(const uint8_t *, const uint8_t *, uint8_t *);

static uint32_t* _XorData(uint32_t* leftSide, uint32_t* rightSide, uint16_t wordCount)
{
    uint32_t* output = leftSide;
    while (wordCount != 0)
    {
        *leftSide ^= *rightSide;
        leftSide++;
        rightSide++;
        wordCount--;
    }

    return output;
}

uint32_t CryptoGenerateKey(uint8_t* generatedKey, uint8_t* generatedKeySize)
{
    uint8_t availableBytes = 0;
    while (availableBytes < CRYPTO_KEY_SIZE)
    {
        sd_rand_application_bytes_available_get(&availableBytes);
        SystickDelayMs(1);
    }

    *generatedKeySize = CRYPTO_KEY_SIZE;
    uint32_t retval = sd_rand_application_vector_get(generatedKey, CRYPTO_KEY_SIZE);

    if (retval == NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES)
        return retval;

    return NRF_SUCCESS;
}

uint32_t CryptoCheckMainKey()
{
    uint8_t generatedKey[CRYPTO_KEY_SIZE];

    memcpy(generatedKey, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, CRYPTO_KEY_SIZE);

    for (uint8_t i=0; i<CRYPTO_KEY_SIZE; ++i)
    {
        if (generatedKey[i] != CHAR_MAX)
            return true;
    }

    return false;
}

uint32_t CryptoGenerateAndStoreMainKey()
{
    uint32_t retval = NRF_SUCCESS;
    uint8_t generatedKey[CRYPTO_KEY_SIZE];
    uint8_t generatedKeySize = 0;

    retval = CryptoGenerateKey(generatedKey, &generatedKeySize);

    if (retval == NRF_ERROR_SOC_RAND_NOT_ENOUGH_VALUES)
        return retval;

    retval = IntFlashUpdatePage(generatedKey, CRYPTO_KEY_SIZE, (uint32_t*)CRYPTO_MAIN_KEY_ADDRESS);

    if (retval != FLASH_OP_SUCCESS)
        return retval;

    return NRF_SUCCESS;
}

uint8_t* CryptoGetMainEncryptionKey()
{
    return mainEncryptionKey;
}

uint8_t* CryptoGetCurrentInitialisingVector()
{
    return currentInitialisingVector;
}


uint32_t CryptoECBEncryptData(uint8_t* dataToEncrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* encryptedData)
{
    // Assert the data size.
    if (dataSize != CRYPTO_KEY_SIZE)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    uint8_t rk[11*16];
    memcpy(rk, key, 16);
    AES_128_keyschedule(key, rk+16);
    AES_128_encrypt(rk, dataToEncrypt, encryptedData);

    return NRF_SUCCESS;
}

uint32_t CryptoECBDecryptData(uint8_t* dataToDecrypt,
                           uint16_t dataSize,
                           uint8_t* key,
                           uint8_t keySize,
                           uint8_t* decryptedData)
{
    // Assert the data size.
    if (dataSize != CRYPTO_KEY_SIZE)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    uint8_t rk[11*16];
    memcpy(rk+160, key, 16);

    AES_128_keyschedule_dec(key, rk);
    AES_128_decrypt(rk, dataToDecrypt, decryptedData);


    return NRF_SUCCESS;
}

uint32_t CryptoCFBEncryptData(uint8_t* dataToEncrypt,
                              uint8_t* initialisingVector,
                              uint8_t* key,
                              uint8_t  keySize,
                              uint8_t* encryptedData,
                              uint32_t dataSize)
{
    // Assert the data size. It should be a multiple of keySize
    if (dataSize % keySize != 0)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    uint16_t blocksCount = dataSize/keySize;
    uint8_t* tmpInitVector = malloc(keySize);

    memcpy(tmpInitVector, initialisingVector, keySize);


    for (uint16_t i=0; i<blocksCount; ++i)
    {
        // Encrypt the initialising vector and store the output in encryptedDatabuffer
        CryptoECBEncryptData(tmpInitVector,
                             keySize,
                             key,
                             keySize,
                             encryptedData);

        // Xor the encrypting output with part of data to be encrypted
        encryptedData = (uint8_t*)_XorData((uint32_t*)encryptedData, (uint32_t*)dataToEncrypt, keySize/sizeof(uint32_t));
        // The encrypted data is an initialising vector for the next data packet
        memcpy(tmpInitVector, encryptedData, keySize);
        // Increase the pointers to point to the next packet
        dataToEncrypt += keySize;
        encryptedData += keySize;
    }

    free(tmpInitVector);
    return NRF_SUCCESS;
}

uint32_t CryptoCFBDecryptData(uint8_t* encryptedData,
                              uint8_t* initialisingVector,
                              uint8_t* key,
                              uint8_t  keySize,
                              uint8_t* decryptedData,
                              uint32_t dataSize)
{
    // Assert the data size. It should be a multiple of keySize
    if (dataSize % keySize != 0)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    uint16_t blocksCount = dataSize/keySize;
    uint8_t* tmpInitVector = malloc(keySize);

    memcpy(tmpInitVector, initialisingVector, keySize);


    for (uint16_t i=0; i<blocksCount; ++i)
    {
        // Encrypt the initialising vector and store the output in encryptedDatabuffer
        CryptoECBEncryptData(tmpInitVector,
                             keySize,
                             key,
                             keySize,
                             decryptedData);

        // Xor the encrypting output with part of data to be encrypted
        decryptedData = (uint8_t*)_XorData((uint32_t*)decryptedData, (uint32_t*)encryptedData, keySize/sizeof(uint32_t));
        // The encrypted data is an initialising vector for the next data packet
        memcpy(tmpInitVector, encryptedData, keySize);
        // Increase the pointers to point to the next packet
        encryptedData += keySize;
        decryptedData += keySize;
    }

    free(tmpInitVector);
    return NRF_SUCCESS;
}



