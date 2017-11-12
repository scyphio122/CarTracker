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
#include <string.h>
#include "arm_math.h"
#include "RTC.h"

static imu_sample_set_t _imuIdleAcceleration;

int16_t                 _imuGyroAxisX[IMU_SAMPLE_BUFFER_SIZE];
int16_t                 _imuGyroAxisY[IMU_SAMPLE_BUFFER_SIZE];
int16_t                 _imuGyroAxisZ[IMU_SAMPLE_BUFFER_SIZE];
int16_t                 _imuAccelerometerAxisX[IMU_SAMPLE_BUFFER_SIZE];
int16_t                 _imuAccelerometerAxisY[IMU_SAMPLE_BUFFER_SIZE];
int16_t                 _imuAccelerometerAxisZ[IMU_SAMPLE_BUFFER_SIZE];

static uint32_t         _imuResultantVectorsLength[IMU_SAMPLE_BUFFER_SIZE];
static uint16_t         _imuSamplesCount;
static bool             _isGyroStarted = false;
static bool             _isGyroInFifo = false;

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
    Rtc1DelayMs(15);

    ImuInitSoftware();

    ImuGetIdleAcceleration();
    ImuSetIdleCorrection(&_imuIdleAcceleration);
}

void ImuTurnOff()
{
    nrf_gpio_pin_set(ACC_ENABLE_PIN);
}

void ImuInitSoftware()
{
    AccelerometerSetLowPower();
    GyroPowerDown();

    AccelerometerSetODR();

//    AccelerometerSetFiltering();

//    ImuEnableDataReadySignal();

    ImuEnableDataRefreshBlockTillPairRead();

    ImuConfigureWakeUpIRQ();
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

void ImuReadRegister(uint8_t address, void* data, uint16_t dataSize)
{
    address |= 0x80;

    SpiEnable(ACC_SPI_PERIPH);
    SpiCSAssert(ACC_CS_PIN);

    SpiSingleWriteContRead(ACC_SPI_PERIPH, address, data, dataSize);

    SpiCSDeassert(ACC_CS_PIN);
    SpiDisable(ACC_SPI_PERIPH);
}

void AccelerometerSetODR()
{
    uint8_t data = 0;

    ImuReadRegister(CTRL1_XL_REG, &data, 1);
    data &= 0x0F;
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
    data &= 0x0F;
    data |= ACC_GYR_ODR_12_5Hz;
    ImuWriteRegister(CTRL2_G_REG, &data, 1);

    _isGyroStarted = true;
}

void GyroPowerDown()
{
    uint8_t data = 0;
    ImuReadRegister(CTRL2_G_REG, &data, 1);
    data |= ACC_ODR_POWER_DOWN;
    ImuWriteRegister(CTRL2_G_REG, &data, 1);

    _isGyroStarted = false;
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
void ImuEnableDataRefreshBlockTillPairRead()
{
    uint8_t data = 0;
    ImuReadRegister(CTRL3_C_REG, &data, 1);
    data |= CTRL3_BLOCK_DATA_UPDATE_SYNCHRONEOUS;
    ImuWriteRegister(CTRL3_C_REG, &data, 1);
}

void ImuEnableDataReadySignal()
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

void ImuGetIdleAcceleration()
{
    ImuReadRegister(OUT_X_G_L, (uint8_t*)&_imuIdleAcceleration, sizeof(imu_sample_set_t));
}

void ImuSetIdleCorrection(imu_sample_set_t* idleSample)
{
    int8_t correctionX = -1 * idleSample->acc_x/256;    //< X axis is internally added by the IMU to the X value
    int8_t correctionY = -1 * idleSample->acc_y/256;    //< Y axis is internally added by the IMU to the Y value
    int8_t correctionZ = idleSample->acc_z/256;         //< Z axis is internally subtracted by the IMU to the Z value

    uint8_t ctrl6Reg = 0;
    ImuReadRegister(CTRL6_C_REG, &ctrl6Reg, sizeof(ctrl6Reg));
    ctrl6Reg &= 0xF3;
    ctrl6Reg |= CTRL6_USR_OFFSET_REG_6BIT_LSB;
    ImuWriteRegister(CTRL6_C_REG, &ctrl6Reg, sizeof(ctrl6Reg));

    ImuWriteRegister(X_OFS_USR_REG, &correctionX, sizeof(correctionX));
    ImuWriteRegister(Y_OFS_USR_REG, &correctionY, sizeof(correctionY));
    ImuWriteRegister(Z_OFS_USR_REG, &correctionZ, sizeof(correctionZ));
}

/* ####################################################################################################################################### */
/*                                                          DETECT TRACK START                                                             */
void ImuSetWakeUpIntPin(uint8_t pinNumber)
{
    uint8_t regValue = 0;
    uint8_t reg = 0;

    if (pinNumber == 1)
    {
        reg = MD1_CFG_REG;
    }
    else
    {
        reg = MD2_CFG_REG;
    }

    ImuReadRegister(reg, &regValue, sizeof(regValue));
    regValue |= MD_CFG_WAKEUP_IRQ_EN;
    ImuWriteRegister(reg, &regValue, sizeof(regValue));


    ImuReadRegister(reg, &regValue, sizeof(regValue));
}



void ImuEnableWakeUpIRQ()
{
    uint8_t reg = 0;
    ImuReadRegister(TAP_CFG_REG, &reg, sizeof(reg));
    reg |=  TAP_CFG_FUNC_IRQ_EN | TAP_CFG_FUNC_IRQ_LATCH;
    ImuWriteRegister(TAP_CFG_REG, &reg, sizeof(reg));


    ImuReadRegister(TAP_CFG_REG, &reg, sizeof(reg));

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

    reg &= 0x08;

    if (reg > 0)
        return true;

    return false;
}

void ImuSetWakeUpIrqThreshold(uint8_t threshold)
{
    threshold &= 0b00111111; //< Mask Single/Double Tap bit and required 0 bit

    ImuWriteRegister(WAKE_UP_THRESH_REG, &threshold, sizeof(threshold));


    ImuReadRegister(WAKE_UP_THRESH_REG, &threshold, sizeof(threshold));
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
    regValue = 0;


    ImuReadRegister(WAKE_UP_DUR_REG, &regValue, sizeof(regValue));
}

void ImuConfigureIrqPinState(uint8_t irq_pin_state_)
{
    uint8_t regValue = 0;
    ImuReadRegister(CTRL3_C_REG, &regValue, sizeof(regValue));
    regValue &= ~0x20;
    regValue |= (irq_pin_state_ & 0x20);
    ImuWriteRegister(CTRL3_C_REG, &regValue, sizeof(regValue));
}

void ImuConfigureWakeUpIRQ()
{
    ImuSetWakeUpIrqThreshold(WAKEUP_ACC_THRESHOLD);//);
    ImuSetWakeUpIrqTriggerSamplesCount(2);

    ImuEnableWakeUpIRQ();

    RTCDelay(NRF_RTC1, RTC1_MS_TO_TICKS(10));

    ImuConfigureIrqPinState(IMU_IRQ_PIN_STATE_HI_TO_LO);

    // Just to discard the first sample
    ImuIsWakeUpIRQ();
//    ImuSetWakeUpIntPin(1);

    ImuSetWakeUpIntPin(2);
}


/* ####################################################################################################################################### */
/*                                                          FIFO CONFIGURATION                                                             */

void ImuSetFifoIntThreshold()
{
    uint16_t threshold = 0x0600;

    ImuWriteRegister(FIFO_CTRL1_REG, (uint8_t*)&threshold, sizeof(threshold));
}

void ImuSetFifoDecimationODR(uint8_t decimationGyro, uint8_t decimationAcc)
{
    uint8_t reg = 0;
    ImuReadRegister(FIFO_CTRL3_REG, &reg, sizeof(reg));

    reg &= 0b11000000;
    reg |= decimationGyro | decimationAcc;

    if (decimationGyro)
    {
        _isGyroInFifo = true;
    }
    else
    {
        _isGyroInFifo = false;
    }

    ImuWriteRegister(FIFO_CTRL3_REG, &reg, sizeof(reg));
}

void ImuSetFifoODR(uint8_t odr)
{
    uint8_t reg = 0;
    ImuReadRegister(FIFO_CTRL5_REG, &reg, sizeof(reg));

    reg &= 0b10000111;
    reg |= odr;

    ImuWriteRegister(FIFO_CTRL5_REG, &reg, sizeof(reg));
}

void ImuSetFifoMode(uint8_t mode)
{
    uint8_t reg = 0;
    ImuReadRegister(FIFO_CTRL5_REG, &reg, sizeof(reg));

    reg &= 0b11111000;
    reg |= mode;

    ImuWriteRegister(FIFO_CTRL5_REG, &reg, sizeof(reg));
}

/**
 * @brief This function configures the internal IMU fifo in the FIFO mode (single buffer, not continuous)
 */
void ImuFifoConfigure()
{
    ImuSetFifoMode(FIFO_MODE_BYPASS);

    // Enable both GYRO and ACC data in the fifo with no decimation
    ImuSetFifoDecimationODR(FIFO_DECIMATION_NO_DECIMATION_GYRO,
                            FIFO_DECIMATION_NO_DECIMATION_ACC);

    ImuSetFifoODR(FIFO_ODR_12_5Hz);
}


void ImuFifoFlush()
{
    // Set to Bypass mode
    ImuSetFifoMode(FIFO_MODE_BYPASS);
    // Set to Fifo mode
    ImuSetFifoMode(FIFO_MODE_FIFO);
}

void ImuFifoStop()
{
    ImuSetFifoMode(FIFO_MODE_BYPASS);
}

void ImuFifoStart()
{
    ImuFifoFlush();
}

uint16_t ImuFifoGetSamplesCount()
{
    uint16_t samplesCount = 0;
    uint8_t divider = 1;

    ImuReadRegister(FIFO_STATUS1, &samplesCount, sizeof(samplesCount));

    if (_isGyroInFifo)
        divider = sizeof(imu_sample_set_t)/sizeof(int16_t);
    else
    {
        divider = sizeof(imu_sample_set_t)/(2*sizeof(int16_t));
    }
    return samplesCount/divider;
}

void ImuFifoReadSingleSampleFromFifo(imu_sample_set_t* sample)
{
    // Read the order Gx, Gy, Gz, Ax, Ay, Az
    if (_isGyroInFifo)
    {
        ImuReadRegister(FIFO_DATA_OUT_L, &_imuGyroAxisX[_imuSamplesCount], sizeof(int16_t));
        ImuReadRegister(FIFO_DATA_OUT_L, &_imuGyroAxisY[_imuSamplesCount], sizeof(int16_t));
        ImuReadRegister(FIFO_DATA_OUT_L, &_imuGyroAxisZ[_imuSamplesCount], sizeof(int16_t));
    }

    ImuReadRegister(FIFO_DATA_OUT_L, &_imuAccelerometerAxisX[_imuSamplesCount], sizeof(int16_t));
    ImuReadRegister(FIFO_DATA_OUT_L, &_imuAccelerometerAxisY[_imuSamplesCount], sizeof(int16_t));
    ImuReadRegister(FIFO_DATA_OUT_L, &_imuAccelerometerAxisZ[_imuSamplesCount], sizeof(int16_t));

    if (sample != NULL)
    {
        memset(sample, 0, sizeof(imu_sample_set_t));

        if (_isGyroInFifo)
        {
            sample->gyro_x = _imuGyroAxisX[_imuSamplesCount];
            sample->gyro_y = _imuGyroAxisY[_imuSamplesCount];
            sample->gyro_z = _imuGyroAxisZ[_imuSamplesCount];
        }
        sample->acc_x = _imuAccelerometerAxisX[_imuSamplesCount];
        sample->acc_y = _imuAccelerometerAxisY[_imuSamplesCount];
        sample->acc_z = _imuAccelerometerAxisX[_imuSamplesCount];
    }
    return;
}

uint16_t ImuFifoGetAllSamples(imu_sample_set_t* optionalSampleArray, uint16_t optionalSampleArraySize)
{
    uint16_t samplesCount = ImuFifoGetSamplesCount();

    if (optionalSampleArray != NULL && samplesCount > optionalSampleArraySize)
    {
        samplesCount = optionalSampleArraySize;
    }

    _imuSamplesCount = 0;

    nrf_gpio_pin_set(DEBUG_ORANGE_LED_PIN);
    for(_imuSamplesCount=0; _imuSamplesCount<samplesCount; ++_imuSamplesCount)
    {
        ImuFifoReadSingleSampleFromFifo(optionalSampleArray);
    }
    nrf_gpio_pin_clear(DEBUG_ORANGE_LED_PIN);

    return samplesCount;
}

/* ####################################################################################################################################### */
/*                                                                CALCULATE ACCELERATIONS                                                  */

int32_t ImuCalculateResultantVector3DLength(int16_t x, int16_t y, int16_t z)
{
    int32_t x_2 = x*x;
    int32_t y_2 = y*y;
    int32_t z_2 = z*z;

    int32_t result = 0;
    arm_sqrt_q31(x_2 +y_2 + z_2, (q31_t*)&result);

    return result;
}

int32_t ImuCalculateMeanValue(void* vector, uint32_t vectorSize, uint8_t wordLength)
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

int32_t ImuGetMeanResultantAccelerationValueFromReadSamples()
{
    int32_t meanAxisX = ImuCalculateMeanValue(_imuAccelerometerAxisX, _imuSamplesCount, sizeof(int16_t));
    int32_t meanAxisY = ImuCalculateMeanValue(_imuAccelerometerAxisY, _imuSamplesCount, sizeof(int16_t));
    int32_t meanAxisZ = ImuCalculateMeanValue(_imuAccelerometerAxisZ, _imuSamplesCount, sizeof(int16_t));
    int32_t totalAcc = ImuCalculateResultantVector3DLength(meanAxisX, meanAxisY, meanAxisZ);

    return totalAcc;
}
