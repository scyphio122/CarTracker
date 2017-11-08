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
#include <stdbool.h>
#include "internal_flash.h"
#include "Systick.h"
#include <string.h>
#include "arm_math.h"
#include "RTC.h"

static imu_sample_set_t _imuIdleAcceleration;
static imu_sample_set_t _imuSampleBuffer[IMU_SAMPLE_BUFFER_SIZE];
static uint32_t         _imuResultantVectorsLength[IMU_SAMPLE_BUFFER_SIZE];
static uint16_t         _imuSampleIndex;

void ImuInit()
{
    nrf_gpio_cfg_input(ACC_INTERRUPT_1_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(ACC_INTERRUPT_2_PIN, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_output(ACC_ENABLE_PIN);
    ImuTurnOff();

    SpiConfig(ACC_SPI_PERIPH,
            SPI_FREQUENCY_FREQUENCY_M8,
            SPI_CONFIG_ORDER_MsbFirst,
            SPI_CONFIG_CPHA_Trailing,
            SPI_CONFIG_CPOL_ActiveLow,
            ACC_CS_PIN);
}

void ImuTurnOn()
{
    nrf_gpio_pin_clear(ACC_ENABLE_PIN);
    SystickDelayMs(15);
    ImuInitSoftware();
}

void ImuTurnOff()
{
    nrf_gpio_pin_set(ACC_ENABLE_PIN);
}

void ImuWriteRegister(uint8_t address, uint8_t* data, uint16_t dataSize)
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

void ImuReadRegister(uint8_t address, uint8_t* data, uint16_t dataSize)
{
    address |= 0x80;

    SpiEnable(ACC_SPI_PERIPH);
    SpiCSAssert(ACC_CS_PIN);

    SpiSingleWriteContRead(ACC_SPI_PERIPH, address, data, dataSize);

    SpiCSDeassert(ACC_CS_PIN);
    SpiDisable(ACC_SPI_PERIPH);
}

void ImuSetFifoIntThreshold()
{
    uint16_t threshold = 0x0600;

    ImuWriteRegister(FIFO_CTRL1_REG, (uint8_t*)&threshold, sizeof(threshold));
}

void ImuSetFifoModeAndOutputRate()
{
    uint8_t ctrl = (0b00010000) | (0b110);
    ImuWriteRegister(FIFO_CTRL5_REG, &ctrl, sizeof(ctrl));
}

void AccelerometerSetODR()
{
    uint8_t data = 0;

    ImuReadRegister(CTRL1_XL_REG, &data, 1);
    data |= ACC_GYR_ODR_12_5Hz;
    ImuWriteRegister(CTRL1_XL_REG, &data, 1);
}

void AccelerometerPowerDown()
{
    uint8_t data = 0;
    ImuReadRegister(CTRL1_XL_REG, &data, 1);
    data |= ACC_ODR_POWER_DOWN;
    ImuWriteRegister(CTRL1_XL_REG, &data, 1);
}

void GyroSetODR()
{
    uint8_t data = 0;
    ImuReadRegister(CTRL2_G_REG, &data, 1);
    data |= ACC_GYR_ODR_12_5Hz;
    ImuWriteRegister(CTRL2_G_REG, &data, 1);
}

void GyroPowerDown()
{
    uint8_t data = 0;
    ImuReadRegister(CTRL2_G_REG, &data, 1);
    data |= ACC_ODR_POWER_DOWN;
    ImuWriteRegister(CTRL2_G_REG, &data, 1);
}

void AccelerometerSetLowPower()
{
    uint8_t data = 0;

    ImuReadRegister(CTRL6_C_REG, &data, 1);
    data |= CTRL6_XL_HM_MODE_DISABLED;
    ImuWriteRegister(CTRL6_C_REG, &data, 1);
}

void AccelerometerSetHighPerformance()
{
    uint8_t data = 0;

    ImuReadRegister(CTRL6_C_REG, &data, 1);
    data &= ~(CTRL6_XL_HM_MODE_DISABLED);
    ImuWriteRegister(CTRL6_C_REG, &data, 1);
}

void AccelerometerSetFiltering()
{
    uint8_t data;

    ImuReadRegister(CTRL1_XL_REG, &data, 1);
    data |= CTRL1_LPF1_BW_SEL;
    ImuWriteRegister(CTRL1_XL_REG, &data, 1);
}

void GyroSetLowPower()
{
    uint8_t data = 0x00;
    ImuReadRegister(CTRL7_G_REG, &data, 1);
    data |= CTRL7_GYRO_HIGH_PERFORMANCE_DISABLED;
    ImuWriteRegister(CTRL7_G_REG, &data, 1);
}

void GyroSetHighPerformance()
{
    uint8_t data = 0x00;
    ImuReadRegister(CTRL7_G_REG, &data, 1);
    data &= ~(CTRL7_GYRO_HIGH_PERFORMANCE_DISABLED);
    ImuWriteRegister(CTRL7_G_REG, &data, 1);
}

uint8_t ImuReadStatusReg()
{
    uint8_t statusReg = 0;
    ImuReadRegister(STATUS_REG, &statusReg, 1);

    return statusReg;
}

/**
 * @brief This function enables the BDU function. The Accelerometer does not refresh the sample in Gyro, Acc or temperature
 * until BOTH of the MSB and LSB in the pair (REG_H and REG_L) are read
 */
static void ImuEnableDataRefreshBlockTillPairRead()
{
    uint8_t data = 0;
    ImuReadRegister(CTRL3_C_REG, &data, 1);
    data |= CTRL3_BLOCK_DATA_UPDATE_SYNCHRONEOUS;
    ImuWriteRegister(CTRL3_C_REG, &data, 1);
}

void ImuEnableDataReadyHardwareIRQ()
{
    // Enable ACC DRDY signal on INT1
    uint8_t data = 0;
    ImuReadRegister(INT1_CTRL_REG, &data, 1);
    data |= INT_DRDY_ACC_EN;
    ImuWriteRegister(INT1_CTRL_REG, &data, 1);

    // Enable GYRO DRDY signal on INT1
    data = 0;
    ImuReadRegister(INT2_CTRL_REG, &data, 1);
    data |= INT_DRDY_GYRO_EN;
    ImuWriteRegister(INT2_CTRL_REG, &data, 1);

    // Set the interupt signal as a pulse rather than continuous high level
    data = DRDY_PULSE_CFG_PULSE_MODE;
    ImuWriteRegister(DRDY_PULSE_CFG, &data, 1);

    ImuReadRegister(CTRL4_C_REG, &data, 1);
    data |= CTRL4_DEN_DRDY_INT1_ENABLE | CTRL4_I2C_DISABLE;       //< Disable I2C and enable DRDY signal masking during setup of filters
    ImuWriteRegister(CTRL4_C_REG, &data, 1);
}

void ImuInitSoftware()
{
    AccelerometerSetLowPower();
    GyroSetLowPower();

    AccelerometerSetODR();
    GyroPowerDown();

    AccelerometerSetFiltering();

//    ImuEnableDataReadyHardwareIRQ();

    ImuEnableDataRefreshBlockTillPairRead();

    ImuConfigureWakeUpIRQ();
}

void ImuGetSample()
{
    ImuReadRegister(OUT_X_G_L, (uint8_t*)&_imuSampleBuffer[_imuSampleIndex], sizeof(imu_sample_set_t));
    _imuSampleIndex++;
}

void ImuGetIdleAcceleration()
{
    ImuReadRegister(OUT_X_G_L, (uint8_t*)&_imuIdleAcceleration, sizeof(imu_sample_set_t));
}

/* ####################################################################################################################################### */
/*                                                          DETECT TRACK START                                                             */
void ImuSetWakeUpPinInt2()
{
    uint8_t reg = 0;
    ImuReadRegister(MD2_CFG_REG, &reg, sizeof(reg));
    reg |= MD_CFG_WAKEUP_IRQ_EN;
    ImuWriteRegister(MD2_CFG_REG, &reg, sizeof(reg));
}

void ImuEnableWakeUpIRQ()
{
    uint8_t reg = 0;
    ImuReadRegister(TAP_CFG_REG, &reg, sizeof(reg));
    reg |= TAP_CFG_FUNC_IRQ_EN | TAP_CFG_FUNC_IRQ_LATCH;
    ImuWriteRegister(TAP_CFG_REG, &reg, sizeof(reg));
}

void ImuDisableWakeUpIRQ()
{
    uint8_t reg = 0;
    ImuReadRegister(TAP_CFG_REG, &reg, sizeof(reg));
    reg &= ~TAP_CFG_FUNC_IRQ_EN;
    ImuWriteRegister(TAP_CFG_REG, &reg, sizeof(reg));
}

bool ImuIsWakeUpIRQ()
{
    uint8_t reg = 0;

    ImuReadRegister(WAKE_UP_SRC_REG, &reg, sizeof(reg));

    reg &= 0x0F;

    if (reg > 0)
        return true;

    return false;
}

void ImuSetWakeUpIrqThreshold(uint8_t threshold)
{
    threshold &= 0b00111111; //< Mask Single/Double Tap bit and required 0 bit

    ImuWriteRegister(WAKE_UP_THRESH_REG, &threshold, sizeof(threshold));
}

/**
 *
 * @param xlOdrCycles Possible values: {0, 1, 2, 3}
 */
void ImuSetWakeUpIrqTriggerSamplesCount(uint8_t xlOdrCycles)
{
    uint8_t regValue = 0;

    xlOdrCycles &= 0x03;

    ImuReadRegister(WAKE_UP_DUR_REG, &regValue, sizeof(regValue));
    regValue &= ~(0x9F);
    regValue |= (xlOdrCycles << 5);
    ImuWriteRegister(WAKE_UP_DUR_REG, &regValue, sizeof(regValue));
}


void ImuConfigureWakeUpIRQ()
{
    ImuSetWakeUpIrqThreshold(WAKEUP_ACC_THRESHOLD);
    ImuSetWakeUpIrqTriggerSamplesCount(2);

    ImuEnableWakeUpIRQ();

    RTCDelay(NRF_RTC1, RTC1_MS_TO_TICKS(10));

    ImuSetWakeUpPinInt2();
}

/* ####################################################################################################################################### */
/*                                                                CALCULATE ACCELERATIONS                                                  */

static int32_t _CalculateResultantVector3DLength(int16_t x, int16_t y, int16_t z)
{
    int32_t x_2 = x*x;
    int32_t y_2 = y*y;
    int32_t z_2 = z*z;

    int32_t result = 0;
    arm_sqrt_q31(x_2 +y_2 + z_2, (q31_t*)&result);

    return result;
}

static int32_t _CalculateMeanValue(void* vector, uint32_t vectorSize, uint8_t wordLength)
{
    int32_t result = 0;
    switch(wordLength)
    {
        case 1:
        {
            arm_mean_q7((q7_t*)vector, vectorSize, (q7_t*)&result);
        }break;

        case 2:
        {
            arm_mean_q15((q15_t*)vector, vectorSize, (q15_t*)&result);
        }break;

        case 4:
        {
            arm_mean_q31((q31_t*)vector, vectorSize, (q31_t*)&result);

        }break;
    }

    return result;
}

int32_t ImuGetMeanResultantAccelerationValue()
{
    uint16_t samplesCount = _imuSampleIndex - 1;
    imu_sample_set_t* samplePtr = _imuSampleBuffer;
    for (uint16_t i=0; i<samplesCount; ++i)
    {
        _imuResultantVectorsLength[i] = _CalculateResultantVector3DLength(samplePtr->acc_x - _imuIdleAcceleration.acc_x,
                                                                         samplePtr->acc_y - _imuIdleAcceleration.acc_y,
                                                                         samplePtr->acc_z - _imuIdleAcceleration.acc_z);
        samplePtr++;
    }

    return _CalculateMeanValue(_imuResultantVectorsLength, samplesCount, sizeof(int32_t));
}
