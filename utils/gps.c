/*
 * gps.c
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#include "gps.h"
#include "at_commands.h"
#include "gsm.h"
#include "UART.h"

uint8_t gpsBuffer[512];

void GpsPowerOn()
{
    GsmUartSendCommand(AT_GPS_POWER_ON, sizeof(AT_GPS_POWER_ON)-1);
}

void GpsPowerOff()
{
    GsmUartSendCommand(AT_GPS_POWER_OFF, sizeof(AT_GPS_POWER_OFF)-1);

}

void GpsGetData()
{
    GsmUartSendCommand(AT_GPS_GET_NAVI_DATA, sizeof(AT_GPS_GET_NAVI_DATA)-1);
    UartReadDataWithPatternSync(gpsBuffer, "OK", 2);
}
