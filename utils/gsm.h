/*
 * gsm.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_GSM_H_
#define UTILS_GSM_H_

#include <stdint-gcc.h>

typedef enum
{
    GSM_OK,
    GSM_ERROR
}gsm_error_e;

#define GSM_FIXED_BAUDRATE_SET      (uint32_t)(0xFFFFFFFE)

void GsmGpsInit();

void GsmBatteryOn();

void GsmBatteryOff();

void GsmPowerOn();

void GsmPowerOff();

gsm_error_e GsmUartSendCommand(void* command, uint16_t commandSize);

void GsmBlockIncommingCalls();

void GsmSmsInit();

void GsmSmsSend(char* telNum, const char* text);



#endif /* UTILS_GSM_H_ */
