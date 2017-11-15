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

#define GSM_SECUCAR_SERVER_IP                   "89.77.116.157"
#define GSM_SECUCAR_SERVER_PORT                 "8080"
#define GSM_SECUCAR_SERVER_BASE_URL             "http://" GSM_SECUCAR_SERVER_IP ":" GSM_SECUCAR_SERVER_PORT "/"
#define GSM_SECUCAR_SERVER_BASE_URL_SIZE        sizeof(GSM_SECUCAR_SERVER_BASE_URL)

#define GSM_HTTP_INPUT_URL_TIMEOUT              "1"
#define GSM_HTTP_SERVER_RESPONSE_TIMEOUT_SEC    "30"
#define GSM_HTTP_POST_INPUT_FILL_TIMEOUT_SEC    "2"

#define GSM_SMS_COMMAND_GET_LOCATION            "GET LOCATION"
#define GSM_SMS_COMMAND_ABORT_ALARM             "ABORT ALARM"

typedef enum
{
    GSM_OK,
    GSM_ERROR,
    GSM_TIMEOUT,
    GSM_ERROR_HTTP_SERVER_INTERNAL,
    GSM_ERROR_HTTP_DAMAGED_RESPONSE
}gsm_error_e;

extern uint64_t    gsmDeviceNumber;
extern uint64_t    gsmOwnerDeviceNumber;
extern uint32_t    deviceId;

#define GSM_FIXED_BAUDRATE_SET      (uint32_t)(0xFFFFFFFE)

void GsmGpsPinsInit();

void GsmGpsInit();

void GsmBatteryOn();

void GsmBatteryOff();

void GsmPowerOn(bool waitForLogon);

void GsmPowerOff();

gsm_error_e GsmUartSendCommand(void* command, uint16_t commandSize, char* response);

gsm_error_e GsmUartSendCommandWithDifferentResponse(void* command, uint16_t commandSize, char* response, char* successResponse);

void GsmBlockIncommingCalls();

void GsmSmsInit();

void GsmSmsSend(char* telNum, const char* text);

void GsmSmsReadAll();

void GsmSmsDelete(int smsIndex);

void GsmSmsDeleteAll();

void GsmSynchronizeTime();

gsm_error_e GsmGprsEnable();

gsm_error_e GsmGprsDisable();

gsm_error_e GsmHttpSendGet(uint8_t* relativeUrl);

gsm_error_e GsmHttpSendPost(uint8_t* relativeUrl, uint8_t* data, uint32_t dataSize);

gsm_error_e GsmHttpGetServerResponse(uint8_t* buf);

gsm_error_e GsmHttpSendStartTrack();

gsm_error_e GsmHttpSendSample(gps_sample_t* sample);

gsm_error_e GsmHttpEndTrack();

#endif /* UTILS_GSM_H_ */
