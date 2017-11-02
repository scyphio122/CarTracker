/*
 * ext_flash.c
 *
 *  Created on: 24 gru 2015
 *      Author: Konrad
 */

#include "external_flash_driver.h"
#include "pinout.h"
#include <nrf_error.h>
#include <nrf52.h>
#include "RTC.h"
#include "nrf_gpio.h"
#include "SPI.h"
#include <sys/_stdint.h>
#include <stdlib.h>
#include <string.h>

static uint8_t                  ext_flash_on = 0;
static uint8_t*                 ext_flash_data_ptr = NULL;
static uint16_t                 ext_flash_data_size;
static ext_flash_status_reg_t   ext_flash_status_reg = {0};

static void ExtFlashSpiInit()
{
    SpiConfig(EXT_FLASH_SPI_PERIPH,
              SPI_FREQUENCY_FREQUENCY_M8,
              SPI_CONFIG_ORDER_MsbFirst,
              SPI_CONFIG_CPHA_Leading,
              SPI_CONFIG_CPOL_ActiveHigh,
              EXT_FLASH_CS_PIN
            );

}

uint32_t ExtFlashInit()
{
    nrf_gpio_cfg_input(EXT_FLASH_ENABLE_PIN, NRF_GPIO_PIN_PULLUP);
//    nrf_gpio_pin_set(EXT_FLASH_ENABLE_PIN);

//    ExtFlashSpiInit();


    return NRF_SUCCESS;
}
/**
 * \brief This function turns on the External Flash module. It blocks program execution untill the Read or Program/Erase operation can be done.
 *
 * \param read_or_erase - the param which tells what kind of operation will be done next. It is needed, because the time from power-up to read command or program/erase command is different
 *
 */
__attribute__((optimize("O0")))
uint32_t ExtFlashTurnOn(ext_flash_operation_type_e read_or_erase)
{
    if(ext_flash_on == 0)
    {
        NRF_GPIO->OUTCLR = 1 << EXT_FLASH_ENABLE_PIN;
//        nrf_gpio_pin_clear(EXT_FLASH_ENABLE_PIN);
        ext_flash_on = 1;
        if(read_or_erase == EXT_FLASH_READ_OP)
            RTCDelay(NRF_RTC1, RTC1_US_TO_TICKS(EXT_FLASH_TURN_ON_DELAY_READ_US));
        else
            RTCDelay(NRF_RTC1, RTC1_US_TO_TICKS(EXT_FLASH_TURN_ON_DELAY_PROGRAM_ERASE_US));
        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_STATE;
}

/**
 * \brief This function turns off the external flash module.
 */
__attribute__((optimize("O0")))
uint32_t ExtFlashTurnOff()
{
    if(ext_flash_on == 1)
    {
        NRF_GPIO->OUTSET = 1 << EXT_FLASH_ENABLE_PIN;
        ext_flash_on = 0;

        return NRF_SUCCESS;
    }

    return NRF_ERROR_INVALID_STATE;
}


/**
 * \brief This function reads the single status register
 *
 * \return  NRF_SUCCESS - everything went fine
 *          NRF_ERROR_INVALID_STATE - the flash module is powered down
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashReadStatusReg()
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    uint8_t command_code = EXT_FLASH_STATUS_REG_READ;

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);

    SpiWrite(EXT_FLASH_SPI_PERIPH, &command_code, sizeof(command_code));
    SpiRead(EXT_FLASH_SPI_PERIPH,  (uint8_t*)&ext_flash_status_reg, sizeof(ext_flash_status_reg));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    return NRF_SUCCESS;
}

/**
 * \brief This function blocks the program execution until the external flash is ready to do another operation
 *
 * \return  NRF_SUCCESS - everything went fine
 *          NRF_ERROR_INVALID_STATE - the flash module is powered down
 */
//__attribute__((optimize("O2")))
static uint32_t ExtFlashWaitTillReady()
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;
    do
    {
        ExtFlashReadStatusReg();
        RTCDelay(NRF_RTC1, RTC1_MS_TO_TICKS(1));
    }while(ext_flash_status_reg.ready_or_busy != 1);

    return NRF_SUCCESS;
}

/**
 * \brief This function checks the value of the erase_or_program_error bit in the ext_flash_status_reg structure.
 *          If the bit is set high after the last Status Register Read operation, an error ocurred, otherwise no error was discovered
 *
 * \return      NRF_SUCCESS - no error ocurred
 *              NRF_ERROR_INTERNAL - error occurred
 *
 */
__attribute__((optimize("O2")))
static inline  uint32_t ExtFlashCheckProgramEraseError()
{
    if(ext_flash_status_reg.erase_or_program_error == 1)
        return NRF_ERROR_INTERNAL;

    return NRF_SUCCESS;
}

/**
 * \brief This function stores the data in the specified Ext Flash SRAM buffer.
 *          It then can be flashed using the Ext_Flash_Program_Page_With_Preerase() or Ext_Flash_Program_Page_Without_Preerase() functions
 *
 * \param buffer_number - number of the buffer where the data is to be stored
 * \param buffer_addrress - the start addrress, where the data is to be stored inside the buffer
 * \param data - pointer to the data buffer
 * \param data_size - size of data which is to be stored in the buffer
 *
 * \return  NRF_SUCCESS - everything went fine
 *          NRF_ERROR_INVALID_STATE - the flash module is powered down
 *          NRF_ERROR_DATA_SIZE - the data exceeds outside the buffer size
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashWriteBuffer(ext_flash_buffer_number_e buffer_number, uint8_t buffer_address, uint8_t* data, uint16_t data_size)
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    if((buffer_address + data_size) > (buffer_address + EXT_FLASH_PAGE_SIZE))
        return NRF_ERROR_DATA_SIZE;

    uint8_t* data_ptr = malloc(data_size + 4);

    /// Set the buffer addrress
    if(buffer_number == EXT_FLASH_BUFFER_1)
        data_ptr[0] = EXT_FLASH_WRITE_BUFFER_1;
    else
        data_ptr[0] = EXT_FLASH_WRITE_BUFFER_2;

    /// Set the 2 dummy bytes and buffer addrress
    data_ptr[1] = 0;
    data_ptr[2] = 0;
    data_ptr[3] = buffer_address;

    /// Copy the data
    memcpy(&data_ptr[4], data, data_size);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);

    /// Send the data
    SpiWrite(EXT_FLASH_SPI_PERIPH, data_ptr, data_size + 4);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    free(data_ptr);

    return NRF_SUCCESS;
}

/**
 * \brief This function flashes the data which is stored in the ext_flash module's SRAM buffer on the page with the given addrress.
 *
 *      IT PRE_ERASES THE FLASH PAGE BEFORE DATA FLASHING
 *
 * NOTE: The flash page addrress must be aligned to the flash page start addrress (lower 8 bits are treated as dummy bits)
 *
 * \param buf_number - number of the buffer from which the data is to be flashed
 * \param addrress - the addrress of the flash page where the data is to be stored. It must be aligned to the flash page start addrress
 *
 * \return  NRF_ERROR_INVALID_STATE - the ext_flash_module is turned off
 *          NRF_SUCCESS - everything went fine
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashProgramPageWithPreerase(ext_flash_buffer_number_e buf_number, uint32_t address)
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    uint8_t addr [4];

    if(buf_number == EXT_FLASH_BUFFER_1)
        addr[0] = EXT_FLASH_PAGE_PROG_FROM_BUF_W_PREERASE_BUF_1;
    else
        addr[0] = EXT_FLASH_PAGE_PROG_FROM_BUF_W_PREERASE_BUF_2;

    addr[1] = (uint8_t)(address >> 16);
    addr[2] = (uint8_t)(address >> 8);
    addr[3] = (uint8_t)address;


    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Program the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, addr, sizeof(addr));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function flashes the data which is stored in the ext_flash module's SRAM buffer on the page with the given addrress.
 *
 *      IT DOES NOT ERASE THE FLASH PAGE BEFORE DATA FLASHING
 *
 * NOTE: The flash page addrress must be aligned to the flash page start addrress (lower 8 bits are treated as dummy bits)
 *
 * \param buf_number - number of the buffer from which the data is to be flashed
 * \param addrress - the addrress of the flash page where the data is to be stored. It must be aligned to the flash page start addrress
 *
 * \return  NRF_ERROR_INVALID_STATE - the ext_flash_module is turned off
 *          NRF_SUCCESS - everything went fine
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashProgramPageWithoutPreerase(ext_flash_buffer_number_e buf_number, uint32_t address)
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    uint8_t addr [4];

    if(buf_number == EXT_FLASH_BUFFER_1)
        addr[0] = EXT_FLASH_PAGE_PROG_FROM_BUF_WITHOUT_PREERASE_BUF_1;
    else
        addr[0] = EXT_FLASH_PAGE_PROG_FROM_BUF_WITHOUT_PREERASE_BUF_2;

    addr[1] = (uint8_t)(address >> 16);
    addr[2] = (uint8_t)(address >> 8);
    addr[3] = (uint8_t)address;

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, addr, sizeof(addr));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function programs direclty the flash page with preerase. It is an combination of buffer write and buffer to main page with preerase operation
 *
 * \param buf_number - number of buffer through which the data will flow
 * \param address - the address in the memory where the data will be stored
 * \param data - pointer to the buffer where data are stored
 * \param data_size - size of data to be stored
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashProgramPageThroughBufferWPreerase(ext_flash_buffer_number_e buf_number, uint32_t address, uint8_t *data, uint16_t data_size)
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    uint8_t* data_ptr = malloc(data_size + 4);

    /// Set the buffer addrress
    if(buf_number == EXT_FLASH_BUFFER_1)
        data_ptr[0] = EXT_FLASH_PAGE_PROG_THROUGH_BUF_W_PRERASE_BUF_1;
    else
        data_ptr[0] = EXT_FLASH_PAGE_PROG_THROUGH_BUF_W_PRERASE_BUF_2;

    /// Set the address
    data_ptr[1] = (uint8_t)(address >> 16);
    data_ptr[2] = (uint8_t)(address >> 8);
    data_ptr[3] = (uint8_t)address;

    /// Copy the data
    memcpy(&data_ptr[4], data, data_size);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, data_ptr, data_size + 4);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function programs direclty the flash page without preerase. It is an combination of buffer write and buffer to main page without preerase operation
 *
 * \param buf_number - number of buffer through which the data will flow
 * \param address - the address in the memory where the data will be stored
 * \param data - pointer to the buffer where data are stored
 * \param data_size - size of data to be stored
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashProgramPageThroughBufferWithoutPreerase(uint32_t address, void *data, uint16_t data_size)
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    uint8_t* data_ptr = malloc(data_size + 4);

    /// Set the buffer addrrss
    data_ptr[0] = EXT_FLASH_PAGE_PROG_THROUGH_BUF_WITHOUT_PRERASE_BUF_1;

    /// Set the address
    data_ptr[1] = (uint8_t)(address >> 16);
    data_ptr[2] = (uint8_t)(address >> 8);
    data_ptr[3] = (uint8_t)address;

    /// Copy the data
    memcpy(&data_ptr[4], data, data_size);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);

    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, data_ptr, data_size + 4);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function update the single or multiple bytes in the page. It is a combination of main memory to buffer, buffer write and buffer to main memory with preerase.
 *
 * \param buf_number - number of buffer through which the data will flow
 * \param address - the address in the memory where the data will be stored
 * \param data - pointer to the buffer where data are stored
 * \param data_size - size of data to be stored
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashUpdateDataOnPage(ext_flash_buffer_number_e buf_number, uint32_t address, uint8_t *data, uint16_t data_size)
{
    if(ext_flash_on == 0)
        return NRF_ERROR_INVALID_STATE;

    uint8_t* data_ptr = malloc(data_size + 4);

    /// Set the buffer addrress
    if(buf_number == EXT_FLASH_BUFFER_1)
        data_ptr[0] = EXT_FLASH_PAGE_UPDATE_BUF_1;
    else
        data_ptr[0] = EXT_FLASH_PAGE_UPDATE_BUF_2;

    /// Set the address
    data_ptr[1] = (uint8_t)(address >> 16);
    data_ptr[2] = (uint8_t)(address >> 8);
    data_ptr[3] = (uint8_t)address;

    /// Copy the data
    memcpy(&data_ptr[4], data, data_size);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);

    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, data_ptr, data_size + 4);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}


/**
 * \brief This function erases entire external flash memory.
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashEraseChip()
{
    uint32_t command = EXT_FLASH_ERASE_CHIP;

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)&command, sizeof(command));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function erases entire sector.
 *
 * \param sector_number - enum encoding sector number
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashEraseSector(ext_flash_sector_numbers_e sector_number)
{
    uint8_t spi_command[4] = {0};
    uint32_t sector_num = 0;

    if(sector_number == EXT_FLASH_SECTOR_0A)
        sector_num = 0;
    else if(sector_number == EXT_FLASH_SECTOR_0B)
        sector_num = 2048;
    else
        sector_num = ((uint32_t)sector_number) << 18;

    spi_command[0] = EXT_FLASH_ERASE_SECTOR;
    spi_command[1] = (uint8_t)(sector_num >> 16);
    spi_command[2] = (uint8_t)(sector_num >> 8);
    spi_command[3] = (uint8_t)sector_num;

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)&spi_command, sizeof(spi_command));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function erases the block which contains 8 pages.
 *
 * \param block_number - the block number, starting from 0
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashEraseBlock(uint16_t block_number)
{
    uint8_t spi_command[4] = {0};
    uint32_t spi_message = block_number << 11;

    spi_command[0] = EXT_FLASH_ERASE_BLOCK_8_PAGES;
    spi_command[1] = (uint8_t)(spi_message >> 16);
    spi_command[2] = (uint8_t)(spi_message >> 8);
    spi_command[3] = (uint8_t)(spi_message);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)&spi_command, sizeof(spi_command));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}


/**
 * \brief This function erases the given page
 *
 * \address - the address within the flash page
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashErasePage(uint32_t address)
{
    uint8_t spi_command[4];

    address = address - (address%256) + 1;
    spi_command[0] = EXT_FLASH_ERASE_PAGE;
    spi_command[1] = (uint8_t)(address >> 16);
    spi_command[2] = (uint8_t)(address >> 8);
    spi_command[3] = (uint8_t)address;

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    /// Start programming the page
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)&spi_command, sizeof(spi_command));

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();
    /// Check if there was an error
    uint32_t err_code = ExtFlashCheckProgramEraseError();

    if(err_code != NRF_SUCCESS)
        return err_code;

    return NRF_SUCCESS;
}

/**
 * \brief This function reads data directly from the external memory single flash page. It bypasses the ext flash module's SRAM buffers, so theirs content is left unchanged
 *
 * \param address - address from which the data is to be read
 * \param data_buf - the buffer for the data read
 * \param data_size - size of data to read
 *
 * \return  NRF_SUCCESS - everything went fine
 *          NRF_ERROR_DATA_SIZE - the user tries to read too much data from the page
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashReadPage(uint32_t address, void* data_buf, uint16_t data_size)
{
    if(((address%EXT_FLASH_PAGE_SIZE) + data_size) > EXT_FLASH_PAGE_SIZE)
        return NRF_ERROR_DATA_SIZE;

    uint8_t spi_command[8] = {0};

    spi_command[0] = EXT_FLASH_READ_PAGE;
    spi_command[1] = (uint8_t)(address >> 16);
    spi_command[2] = (uint8_t)(address >> 8);
    spi_command[3] = (uint8_t)address;

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)&spi_command, sizeof(spi_command));
    SpiRead(EXT_FLASH_SPI_PERIPH, data_buf, data_size);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    return NRF_SUCCESS;
}

/**
 * \brief This function reads data directly from the external memory . It bypasses the ext flash module's SRAM buffers, so theirs content is left unchanged.
 *          If end of the page is reached, the data from the next page is clocked out.
 *
 * \param address - address from which the data is to be read
 * \param data_buf - the buffer for the data read
 * \param data_size - size of data to read
 *
 * \return  NRF_SUCCESS - everything went fine
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashReadContinuous(uint32_t address, uint8_t* data_buf, uint16_t data_size)
{
    uint8_t spi_command[8] = {0};

    spi_command[0] = EXT_FLASH_CONTINUOUS_READ;
    spi_command[1] = (uint8_t)(address >> 16);
    spi_command[2] = (uint8_t)(address >> 8);
    spi_command[3] = (uint8_t)address;

    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)&spi_command, sizeof(spi_command));
    SpiRead(EXT_FLASH_SPI_PERIPH, data_buf, data_size);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);

    return NRF_SUCCESS;
}

/**
 * \brief This function reads the data contained in one of the two SRAM buffers inside the external flash memory chip.
 *
 * \param buf_read_command - command used to read the buffer. It differs because of the buffer number and max read operation speed. Available options:
 *                          ~EXT_FLASH_READ_BUF_MAX_SPEED_BUF_1
 *                          ~EXT_FLASH_READ_BUF_MAX_SPEED_BUF_2
 *                          ~EXT_FLASH_READ_BUF_LOW_SPEED_BUF_1
 *                          ~EXT_FLASH_READ_BUF_LOW_SPEED_BUF_2
 * \param buffer_address - address inside the buffer (0 - 255) which is to be read
 * \param data_buf - buffer for the data
 * \param data_size - size of data to be read
 */
//__attribute__((optimize("O2")))
uint32_t ExtFlashReadBuffer(uint8_t buf_read_command, uint8_t buffer_address, uint8_t* data_buf, uint16_t data_size)
{
    uint8_t* spi_command;
    uint8_t  spi_command_size = 0;
    if((buf_read_command == EXT_FLASH_READ_BUF_MAX_SPEED_BUF_1) || (buf_read_command == EXT_FLASH_READ_BUF_MAX_SPEED_BUF_2))
    {
        spi_command = malloc(5);
        spi_command[0] = buf_read_command;
        spi_command[1] = 0;
        spi_command[2] = 0;
        spi_command[3] = buffer_address;
        spi_command[4] = 0;
        spi_command_size = 5;
    }
    else
    {
        spi_command = malloc(4);
        spi_command[0] = buf_read_command;
        spi_command[1] = 0;
        spi_command[2] = 0;
        spi_command[3] = buffer_address;
        spi_command_size = 4;
    }
    /// Wait until the flash module is ready
    ExtFlashWaitTillReady();

    SpiCSAssert(EXT_FLASH_CS_PIN);
    SpiEnable(EXT_FLASH_SPI_PERIPH);
    SpiWrite(EXT_FLASH_SPI_PERIPH, (uint8_t*)spi_command, spi_command_size);
    SpiRead(EXT_FLASH_SPI_PERIPH, data_buf, data_size);

    SpiCSDeassert(EXT_FLASH_CS_PIN);
    SpiDisable(EXT_FLASH_SPI_PERIPH);
    free(spi_command);

    return NRF_SUCCESS;
}
