/*
 * lsm6dsm.h
 *
 *  Created on: Nov 5, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_LSM6DSM_H_
#define UTILS_LSM6DSM_H_

#include <stdint-gcc.h>


#define ACCELEROMETER_THRESHOLD     INT16_MAX/10    //< About 1.962m/s^2

#define FUNC_CFG_ACCESS_REG  0x01

#define FIFO_CTRL1_REG       0x06
#define FIFO_CTRL2_REG       0x07
#define FIFO_CTRL3_REG       0x08
#define FIFO_CTRL4_REG       0x09
#define FIFO_CTRL5_REG       0x0A

#define DRDY_PULSE_CFG      0x0B

#define INT1_CTRL_REG        0x0D
#define INT2_CTRL_REG        0x0E

#define INT_DRDY_ACC_EN     0x01
#define INT_DRDY_GYRO_EN    0x02

#define WHO_AM_I_REG         0x0F

#define CTRL1_XL_REG         0x10
#define CTRL2_G_REG          0x11
#define CTRL3_C_REG          0x12
#define CTRL4_C_REG          0x13
#define CTRL5_C_REG          0x14
#define CTRL6_C_REG          0x15
#define CTRL7_G_REG          0x16
#define CTRL8_XL_REG         0x17
#define CTRL9_XL_REG         0x18
#define CTRL10_C_REG         0x19

#define WAKE_UP_SRC_REG      0x1B
#define TAP_SRC_REG          0x1C

#define STATUS_REG           0x1E

#define OUT_TEMP_L           0x20
#define OUT_TEMP_H           0x21

#define OUT_X_G_L            0x22
#define OUT_X_G_H            0x23
#define OUT_Y_G_L            0x24
#define OUT_Y_G_H            0x25
#define OUT_Z_G_L            0x26
#define OUT_Z_G_H            0x27

#define OUT_X_XL_L           0x28
#define OUT_X_XL_H           0x29
#define OUT_Y_XL_L           0x2A
#define OUT_Y_XL_H           0x2B
#define OUT_Z_XL_L           0x2C
#define OUT_Z_XL_H           0x2D

#define FIFO_STATUS1        0x3A
#define FIFO_STATUS2        0x3B
#define FIFO_STATUS3        0x3C
#define FIFO_STATUS4        0x3D

#define FIFO_DATA_OUT_L     0x3E
#define FIFO_DATA_OUT_H     0x3F


#define ACC_ODR_POWER_DOWN  0x00
#define ACC_ODR_1_6Hz       0xB0
#define ACC_GYR_ODR_12_5Hz  0x10
#define ACC_GYR_ODR_26Hz    0x20
#define ACC_GYR_ODR_52Hz    0x30
#define ACC_GYR_ODR_104Hz   0x40
#define ACC_GYR_ODR_208Hz   0x50
#define ACC_GYR_ODR_416Hz   0x60
#define ACC_GYR_ODR_833Hz   0x70
#define ACC_GYR_ODR_1666Hz  0x80
#define ACC_GYR_ODR_3333Hz  0x90
#define ACC_GYR_ODR_6666Hz  0xA0

void AccelerometerInit();

void AccTurnOn();

void AccTurnOff();

void AccWriteRegister(uint8_t address, uint8_t* data, uint16_t dataSize);

void AccReadRegister(uint8_t address, uint8_t* data, uint16_t dataSize);

void AccInitSoftware();


void AccSetFifoIntThreshold();

void AccSetFifoModeAndOutputRate();

void AccelerometerWakeUp();

void AccelerometerPowerDown();

void GyroWakeUp();

void GyroPowerDown();

void AccelerometerSetLowPower();

void AccelerometerSetHighPerformance();

void GyroSetLowPower();

void GyroSetHighPerformance();

void AccelerometerSetFiltering();

uint8_t ImuReadStatusReg();

void ImuEnableDataReadyHardwareIRQ();

#endif /* UTILS_LSM6DSM_H_ */
