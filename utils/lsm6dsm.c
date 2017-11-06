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

void AccelerometerWakeUp()
{
    uint8_t data = 0;

    AccReadRegister(CTRL1_XL_REG, &data, 1);
    data |= ACC_GYR_ODR_12_5Hz;
    AccWriteRegister(CTRL1_XL_REG, &data, 1);
}

void AccelerometerPowerDown()
{
    uint8_t data = 0;
    AccReadRegister(CTRL1_XL_REG, &data, 1);
    data |= ACC_ODR_POWER_DOWN;
    AccWriteRegister(CTRL1_XL_REG, &data, 1);
}

void GyroWakeUp()
{
    uint8_t data = 0;
    AccReadRegister(CTRL2_G_REG, &data, 1);
    data |= ACC_GYR_ODR_12_5Hz;
    AccWriteRegister(CTRL2_G_REG, &data, 1);
}

void GyroPowerDown()
{
    uint8_t data = 0;
    AccReadRegister(CTRL2_G_REG, &data, 1);
    data |= ACC_ODR_POWER_DOWN;
    AccWriteRegister(CTRL2_G_REG, &data, 1);
}

void AccelerometerSetLowPower()
{
    uint8_t data = 0;

    AccReadRegister(CTRL6_C_REG, &data, 1);

    data |= 0x10;

    AccWriteRegister(CTRL6_C_REG, &data, 1);
}

void AccelerometerSetHighPerformance()
{
    uint8_t data = 0;

    AccReadRegister(CTRL6_C_REG, &data, 1);

    data &= ~(0x10);

    AccWriteRegister(CTRL6_C_REG, &data, 1);
}

void AccelerometerSetFiltering()
{
    uint8_t data;

    AccReadRegister(CTRL1_XL_REG, &data, 1);

    data |= 0x02;

    AccWriteRegister(CTRL1_XL_REG, &data, 1);
}

void GyroSetLowPower()
{
    uint8_t data = 0x00;
    AccReadRegister(CTRL7_G_REG, &data, 1);

    data |= 0x80;

    AccWriteRegister(CTRL7_G_REG, &data, 1);
}

void GyroSetHighPerformance()
{
    uint8_t data = 0x80;
    AccReadRegister(CTRL7_G_REG, &data, 1);

    data &= ~(0x80);

    AccWriteRegister(CTRL7_G_REG, &data, 1);
}

uint8_t ImuReadStatusReg()
{
    uint8_t statusReg = 0;

    AccReadRegister(STATUS_REG, &statusReg, 1);

    return statusReg;
}

/**
 * @brief This function enables the BDU function. The Accelerometer does not refresh the sample in Gyro, Acc or temperature
 * until BOTH of the MSB and LSB in the pair (REG_H and REG_L) are read
 */
static void ImuEnableDataRefreshBlockTillPairRead()
{
    uint8_t data = 0;
    AccReadRegister(CTRL3_C_REG, &data, 1);

    data |= 0x40;

    AccWriteRegister(CTRL3_C_REG, &data, 1);
}

void ImuEnableDataReadyHardwareIRQ()
{
    // Enable ACC DRDY signal on INT1
    uint8_t data = 0;
    AccReadRegister(INT1_CTRL_REG, &data, 1);
    data |= INT_DRDY_ACC_EN;
    AccWriteRegister(INT1_CTRL_REG, &data, 1);

    // Enable GYRO DRDY signal on INT1
    data = 0;
    AccReadRegister(INT2_CTRL_REG, &data, 1);
    data |= INT_DRDY_GYRO_EN;
    AccWriteRegister(INT2_CTRL_REG, &data, 1);

    // Set the interupt signal as a pulse rather than continuous high level
    data = 0x80;
    AccWriteRegister(DRDY_PULSE_CFG, &data, 1);

    AccReadRegister(CTRL4_C_REG, &data, 1);
    data |= 0x0C;       //< Disable I2C and enable DRDY signal masking during setup of filters
    AccWriteRegister(CTRL4_C_REG, &data, 1);
}

void ImuInitSoftware()
{
    AccelerometerSetLowPower();
    GyroSetLowPower();

    AccelerometerWakeUp();
    GyroPowerDown()();

    AccelerometerSetFiltering();

    ImuEnableDataReadyHardwareIRQ();

    ImuEnableDataRefreshBlockTillPairRead();
}



