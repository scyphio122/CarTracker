/*
 * lsm6dsm.c
 *
 *  Created on: Nov 5, 2017
 *      Author: Konrad Traczyk
 */
#include "lsm6dsm.h"
#include "pinout.h"
#include "SPI.h"
#include "nrf_gpio.h"
#include <stdint-gcc.h>
#include "internal_flash.h"
#include "Systick.h"
#include <string.h>

void AccelerometerInit()
{
    nrf_gpio_cfg_input(ACC_INTERRUPT_1_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(ACC_INTERRUPT_2_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_output(ACC_ENABLE_PIN);
    AccTurnOff();

    SpiConfig(ACC_SPI_PERIPH,
            SPI_FREQUENCY_FREQUENCY_M8,
            SPI_CONFIG_ORDER_MsbFirst,
            SPI_CONFIG_CPHA_Trailing,
            SPI_CONFIG_CPOL_ActiveLow,
            ACC_CS_PIN);
}

void AccTurnOn()
{
    nrf_gpio_pin_clear(ACC_ENABLE_PIN);
    SystickDelayMs(15);
}

void AccTurnOff()
{
    nrf_gpio_pin_set(ACC_ENABLE_PIN);
}

void AccWriteRegister(uint8_t address, uint8_t* data, uint16_t dataSize)
{
    char* buf = malloc(dataSize + 1);
    buf[0] = (address & ~(0x80));
    memcpy(&buf[1], data, dataSize);

    SpiEnable(ACC_SPI_PERIPH);
    SpiCSAssert(ACC_CS_PIN);

    SpiWrite(ACC_SPI_PERIPH, buf, dataSize + 1);

    SpiCSDeassert(ACC_CS_PIN);
    SpiDisable(ACC_SPI_PERIPH);

    free(buf);
}

void AccReadRegister(uint8_t address, uint8_t* data, uint16_t dataSize)
{
    address |= 0x80;

    SpiEnable(ACC_SPI_PERIPH);
    SpiCSAssert(ACC_CS_PIN);


    SpiWrite(ACC_SPI_PERIPH, &address, sizeof(address));
    SpiRead(ACC_SPI_PERIPH, data, dataSize);

    SpiCSDeassert(ACC_CS_PIN);
    SpiDisable(ACC_SPI_PERIPH);
}

void AccSetFifoIntThreshold()
{
    uint16_t threshold = 0x0600;

    AccWriteRegister(FIFO_CTRL1_REG, (uint8_t*)&threshold, sizeof(threshold));
}

void AccSetFifoModeAndOutputRate()
{
    uint8_t ctrl = (0b00010000) | (0b110);
    AccWriteRegister(FIFO_CTRL5_REG, &ctrl, sizeof(ctrl));
}

void AccInitSoftware()
{
    uint8_t data = 0x60;

    AccWriteRegister(CTRL1_XL_REG, &data, 1);

    data = 0x01;
    AccWriteRegister(INT1_CTRL_REG, &data, 1);
}



