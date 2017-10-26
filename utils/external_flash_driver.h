/*
 * ext_flash.h
 *
 *  Created on: 08 Nov 2017
 *      Author: Konrad Traczyk
 */

#ifndef HARDWARE_EXT_FLASH_H_
#define HARDWARE_EXT_FLASH_H_

#include "stdint-gcc.h"

#define EXT_FLASH_PAGE_SIZE                                     (uint16_t)256
#define EXT_FLASH_END_ADDRESS                                   (uint32_t)0x800000
#define EXT_FLASH_TURN_ON_DELAY_READ_US                         (uint8_t)100
#define EXT_FLASH_TURN_ON_DELAY_PROGRAM_ERASE_US                (uint16_t)3100
#define EXT_FLASH_OP_POLL_TIME_MS                               (uint8_t)1


/**         EXTERNAL FLASH COMMANDS         **/

/*  STATUS REG COMMANDS */
#define EXT_FLASH_STATUS_REG_READ                               (uint8_t)0xD7
#define EXT_FLASH_STATUS_REG_WRITE                              (uint8_t)

/*  READ COMMANDS       */
#define EXT_FLASH_READ_PAGE                                     (uint8_t)0xD2
#define EXT_FLASH_READ_BUF_MAX_SPEED_BUF_1                      (uint8_t)0xD4
#define EXT_FLASH_READ_BUF_MAX_SPEED_BUF_2                      (uint8_t)0xD6
#define EXT_FLASH_READ_BUF_LOW_SPEED_BUF_1                      (uint8_t)0xD1
#define EXT_FLASH_READ_BUF_LOW_SPEED_BUF_2                      (uint8_t)0xD3
#define EXT_FLASH_CONTINUOUS_READ                               (uint8_t)0x01


/*  PROGRAM COMMANDS    */
#define EXT_FLASH_WRITE_BUFFER_1                                (uint8_t)0x84
#define EXT_FLASH_WRITE_BUFFER_2                                (uint8_t)0x87

#define EXT_FLASH_PAGE_PROG_FROM_BUF_W_PREERASE_BUF_1           (uint8_t)0x83
#define EXT_FLASH_PAGE_PROG_FROM_BUF_W_PREERASE_BUF_2           (uint8_t)0x86

#define EXT_FLASH_PAGE_PROG_FROM_BUF_WITHOUT_PREERASE_BUF_1     (uint8_t)0x88
#define EXT_FLASH_PAGE_PROG_FROM_BUF_WITHOUT_PREERASE_BUF_2     (uint8_t)0x89

#define EXT_FLASH_PAGE_PROG_THROUGH_BUF_W_PRERASE_BUF_1         (uint8_t)0x82
#define EXT_FLASH_PAGE_PROG_THROUGH_BUF_W_PRERASE_BUF_2         (uint8_t)0x85

#define EXT_FLASH_PAGE_PROG_THROUGH_BUF_WITHOUT_PRERASE_BUF_1   (uint8_t)0x02

#define EXT_FLASH_PAGE_UPDATE_BUF_1                             (uint8_t)0x58
#define EXT_FLASH_PAGE_UPDATE_BUF_2                             (uint8_t)0x59

/*  ERASE COMMANDS      */
#define EXT_FLASH_ERASE_PAGE                                    (uint8_t)0x81
#define EXT_FLASH_ERASE_BLOCK_8_PAGES                           (uint8_t)0x50
#define EXT_FLASH_ERASE_SECTOR                                  (uint8_t)0x7C
#define EXT_FLASH_ERASE_CHIP                                    (uint32_t)0x9A8094C7

typedef enum
{
    EXT_FLASH_READ_OP = 0,
    EXT_FLASH_PROGRAM_OP,
    EXT_FLASH_ERASE_OP
}ext_flash_operation_type_e;

typedef enum
{
    EXT_FLASH_BUFFER_1 = 0,
    EXT_FLASH_BUFFER_2
}ext_flash_buffer_number_e;

#pragma GCC push
#pragma GCC optimize("O0")
typedef struct
{
    uint8_t page_size_config        :1;     /**< 0 - 264 bytes; 1 - 256 bytes */
    uint8_t sector_protect_stat     :1;     /**< 0 - sector protection disabled; 1 - sector protection enabled */
    uint8_t memory_size             :4;     /**< 1111 - 64 MBit */
    uint8_t compare_result          :1;     /**< 0 - main memory page matches buffer data; 1 - main memory page is differen than buffer data */
    uint8_t ready_or_busy           :1;     /**< 0 - device busy; 1 - device ready */

    uint8_t erase_suspend           :1;     /**< 0 - erase not suspended; 1 - erase op suspended */
    uint8_t program_suspend_buf_1   :1;     /**< 0 - the sector is not program suspended from buf 1; 1 - the sector is program suspended from buf 1 */
    uint8_t program_suspend_buf_2   :1;     /**< 0 - the sector is not program suspended from buf 2; 1 - the sector is program suspended from buf 2 */
    uint8_t sector_lockdown_enabled :1;     /**< 0 - sector lockdown disabled; 1 - sector lockdown enabled */
    uint8_t xxx_1                   :1;     /**< not used */
    uint8_t erase_or_program_error  :1;     /**< 0 - the program or erase operation was successful; 1 - en error during program or erase op occured */
    uint8_t xxx_2                   :1;     /**< not used */
    uint8_t ready_or_busy_2         :1;     /**< 0 - device busy; 1 - device ready */

}ext_flash_status_reg_t;

typedef enum
{
    EXT_FLASH_SECTOR_0A = 0xFE,
    EXT_FLASH_SECTOR_0B = 0xFF,
    EXT_FLASH_SECTOR_1  = 1,
    EXT_FLASH_SECTOR_2  = 2,
    EXT_FLASH_SECTOR_3  = 3,
    EXT_FLASH_SECTOR_4  = 4,
    EXT_FLASH_SECTOR_5  = 5,
    EXT_FLASH_SECTOR_6  = 6,
    EXT_FLASH_SECTOR_7  = 7,
    EXT_FLASH_SECTOR_8  = 8,
    EXT_FLASH_SECTOR_9  = 9,
    EXT_FLASH_SECTOR_10 = 10,
    EXT_FLASH_SECTOR_11 = 11,
    EXT_FLASH_SECTOR_12 = 12,
    EXT_FLASH_SECTOR_13 = 13,
    EXT_FLASH_SECTOR_14 = 14,
    EXT_FLASH_SECTOR_15 = 15,
    EXT_FLASH_SECTOR_16 = 16,
    EXT_FLASH_SECTOR_17 = 17,
    EXT_FLASH_SECTOR_18 = 18,
    EXT_FLASH_SECTOR_19 = 19,
    EXT_FLASH_SECTOR_20 = 20,
    EXT_FLASH_SECTOR_21 = 21,
    EXT_FLASH_SECTOR_22 = 22,
    EXT_FLASH_SECTOR_23 = 23,
    EXT_FLASH_SECTOR_24 = 24,
    EXT_FLASH_SECTOR_25 = 25,
    EXT_FLASH_SECTOR_26 = 26,
    EXT_FLASH_SECTOR_27 = 27,
    EXT_FLASH_SECTOR_28 = 28,
    EXT_FLASH_SECTOR_29 = 29,
    EXT_FLASH_SECTOR_30 = 30,
    EXT_FLASH_SECTOR_31 = 31
}ext_flash_sector_numbers_e;
#pragma GCC pop

uint32_t ExtFlashInit();

uint32_t ExtFlashTurnOn(ext_flash_operation_type_e read_or_erase);

uint32_t ExtFlashTurnOff();

uint32_t ExtFlashReadStatusReg();

uint32_t ExtFlashWriteBuffer(ext_flash_buffer_number_e buffer_number, uint8_t buffer_address, uint8_t* data, uint16_t data_size);

uint32_t ExtFlashProgramPageWithPreerase(ext_flash_buffer_number_e buf_number, uint32_t address);

uint32_t ExtFlashProgramPageWithoutPreerase(ext_flash_buffer_number_e buf_number, uint32_t address);

uint32_t ExtFlashProgramPageThroughBufferWithoutPreerase(uint32_t address, void *data, uint16_t data_size);

uint32_t ExtFlashUpdateDataOnPage(ext_flash_buffer_number_e buf_number, uint32_t address, uint8_t *data, uint16_t data_size);

uint32_t ExtFlashEraseChip();

uint32_t ExtFlashEraseSector(ext_flash_sector_numbers_e sector_number);

uint32_t ExtFlashEraseBlock(uint16_t block_number);

uint32_t ExtFlashErasePage(uint32_t address);

uint32_t ExtFlashReadPage(uint32_t address, void* data_buf, uint16_t data_size);

uint32_t ExtFlashReadContinuous(uint32_t address, uint8_t* data_buf, uint16_t data_size);

uint32_t ExtFlashReadBuffer(uint8_t buf_read_command, uint8_t buffer_address, uint8_t* data_buf, uint16_t data_size);
#endif /* HARDWARE_EXT_FLASH_H_ */