/*
 * gsm.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_GSM_H_
#define UTILS_GSM_H_

#include <stdint-gcc.h>
#include <stdbool.h>
#include "gps.h"

#define GSM_SMS_COMMAND_GET_LOCATION        "GET LOCATION"
#define GSM_SMS_COMMAND_ABORT_ALARM         "ABORT ALARM"

typedef enum
{
    GSM_OK,
    GSM_ERROR
}gsm_error_e;

extern uint64_t    gsmDeviceNumber;
extern uint64_t    gsmOwnerDeviceNumber;
extern uint32_t    deviceId;

#define GSM_FIXED_BAUDRATE_SET      (uint32_t)(0xFFFFFFFE)

void GsmGpsInit();

void GsmBatteryOn();

void GsmBatteryOff();

void GsmPowerOn(bool waitForLogon);

void GsmPowerOff();

gsm_error_e GsmUartSendCommand(void* command, uint16_t commandSize, char* response);

void GsmBlockIncommingCalls();

void GsmSmsInit();

void GsmSmsSend(char* telNum, const char* text);

void GsmSmsReadAll();

void GsmSmsDelete(int smsIndex);

void GsmSmsDeleteAll();

gsm_error_e GsmHttpSendMessage(uint8_t* data, uint32_t dataSize);

gsm_error_e GsmHttpSendSample(gps_sample_t* sample);

#endif /* UTILS_GSM_H_ */
