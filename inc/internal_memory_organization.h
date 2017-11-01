/*
 * internal_memory_organization.h
 *
 *  Created on: Oct 4, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#ifndef INC_INTERNAL_MEMORY_ORGANIZATION_H_
#define INC_INTERNAL_MEMORY_ORGANIZATION_H_

#include "crypto.h"

#define INTERNAL_FLASH_PAGE_SIZE                    (uint16_t)0x1000
#define FLASH_MEMORY_END_ADDRESS                    (uint32_t*)0x80000

#define PERSISTENT_CONFIG_PAGE_ADDRESS              (uint32_t)0x78000

#define CRYPTO_MAIN_KEY_ADDRESS                     (PERSISTENT_CONFIG_PAGE_ADDRESS)
#define GSM_BAUDRATE_CONFIG_ADDRESS                 (CRYPTO_MAIN_KEY_ADDRESS + CRYPTO_KEY_SIZE)
#define GSM_DEVICE_PHONE_NUMBER_ADDRESS             GSM_BAUDRATE_CONFIG_ADDRESS + sizeof(uint32_t)
#define GSM_OWNER_PHONE_NUMBER_ADDRESS              GSM_DEVICE_PHONE_NUMBER_ADDRESS + sizeof(uint64_t)

#define DEVICE_ID                                   GSM_OWNER_PHONE_NUMBER_ADDRESS + sizeof(uint64_t)

#define INTERNAL_FLASH_SWAP_PAGE_ADDRESS            (uint32_t*)0x79000

#endif /* INC_INTERNAL_MEMORY_ORGANIZATION_H_ */
