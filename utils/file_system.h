/*
 * file_system.h
 *
 *  Created on: Oct 8, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_FILE_SYSTEM_H_
#define UTILS_FILE_SYSTEM_H_

#include "stdint-gcc.h"
#include "external_flash_driver.h"

#define EXT_FLASH_AVAILABLE                         (uint8_t)1
#define MAX_FLASH_OPERATION_ERROR_COUNTER           (uint8_t)3

typedef struct
{
    uint16_t entry_number;
    uint16_t num_of_samples;
    uint32_t entry_timestamp;
    uint32_t available_samples_count;
    uint32_t driving_assessment;
}mem_org_flash_page_header_t;

#define MEM_ORG_HEADER_OFFSET_ENTRY_NUMBER          (uint8_t)0
#define MEM_ORG_HEADER_OFFSET_NUM_OF_SAMPLES        (uint8_t)2
#define MEM_ORG_HEADER_OFFSET_ENTRY_TIMESTAMP       (uint8_t)4
#define MEM_ORG_HEADER_OFFSET_AVAILABLE_SAMPLES_CNT (uint8_t)8
#define MEM_ORG_HEADER_OFFSET_DRIVING_ASSESSMENT    (uint8_t)12

typedef struct
{
    uint32_t timestamp;
    uint16_t longtitude[2]; // [0] - integer part [1] - fractional part
    uint16_t latitude[2];   // [0] - integer part [1] - fractional part
    uint8_t  speed;
    uint8_t  acceleration;
    uint16_t azimuth;
}mem_org_gps_sample_t;

typedef enum
{
    E_OP_SUCCESS,
    E_NO_MEMORY,
    E_TIMEOUT,
    E_NOT_FOUND
}mem_org_error_code_e;

typedef uint32_t mem_org_key_t;

#ifndef EXT_FLASH_AVAILABLE
    #define MEM_ORG_KEY_AREA_START_ADDRESS              (uint32_t)0x28800
    #define MEM_ORG_KEY_AREA_END_ADDRESS                (uint32_t)0x29000

    #define MEM_ORG_KEY_AREA_KEYS_ON_PAGE               (uint8_t)(INTERNAL_FLASH_PAGE_SIZE - sizeof(mem_org_flash_page_header_t))/4

    #define MEM_ORG_DATA_AREA_START_ADDRESS             MEM_ORG_KEY_AREA_END_ADDRESS
    #define MEM_ORG_DATA_AREA_END_ADDRESS               (uint32_t)0x3F800

    #define MEM_ORG_DATA_SAMPLES_ON_INT_FLASH_PAGE      (uint8_t)42

    #define MEM_ORG_KEY_ADD_SHIFT                       (uint8_t)5
    #define MEM_ORG_KEY_TRACK_NUMBER_SHIFT              (uint8_t)16
#else
    #define MEM_ORG_KEY_AREA_START_ADDRESS              (uint32_t)0
    #define MEM_ORG_KEY_AREA_END_ADDRESS                (uint32_t)0x2000

    #define MEM_ORG_KEY_AREA_KEYS_ON_PAGE               (uint8_t)((EXT_FLASH_PAGE_SIZE - sizeof(mem_org_flash_page_header_t))/sizeof(mem_org_key_t))

    #define MEM_ORG_DATA_AREA_START_ADDRESS             MEM_ORG_KEY_AREA_END_ADDRESS
    #define MEM_ORG_DATA_AREA_END_ADDRESS               (uint32_t)EXT_FLASH_END_ADDRESS

    #define MEM_ORG_DATA_SAMPLES_ON_FLASH_PAGE          (uint8_t)((EXT_FLASH_PAGE_SIZE - sizeof(mem_org_flash_page_header_t))/sizeof(mem_org_gps_sample_t))

    #define MEM_ORG_KEY_ADD_SHIFT                       (uint8_t)5
    #define MEM_ORG_KEY_TRACK_NUMBER_SHIFT              (uint8_t)16
#endif

#define MEM_ORG_KEY_NOT_INITIALIZED                     (uint32_t)(0x80000000)


extern volatile uint8_t         mem_org_track_samples_storage_enabled;
extern uint32_t                 mem_org_gps_sample_storage_interval;

mem_org_error_code_e Mem_Org_Init();
mem_org_error_code_e Mem_Org_Store_Key(uint32_t address_to_data, uint16_t track_number);
uint32_t Mem_Org_Find_Key(uint16_t track_number, uint32_t* key_buf);
uint32_t Mem_Org_Store_Sample(mem_org_gps_sample_t* sample_data);
uint32_t Mem_Org_Track_Start_Storage();
uint32_t Mem_Org_Track_Stop_Storage();
uint32_t Mem_Org_Clear_Tracks_Memory();


mem_org_error_code_e Mem_Org_List_Tracks_Through_BLE();
mem_org_error_code_e Mem_Org_Send_Track_Via_BLE(uint32_t key);



#endif /* UTILS_FILE_SYSTEM_H_ */