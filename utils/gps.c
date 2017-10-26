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


void GpsPowerOn()
{
    GsmUartSendCommand(AT_GPS_POWER_READ, sizeof(AT_GPS_POWER_READ));
    GsmUartSendCommand(AT_GPS_POWER_ON, sizeof(AT_GPS_POWER_ON));
}

void GpsPowerOff()
{
    GsmUartSendCommand(AT_GPS_POWER_OFF, sizeof(AT_GPS_POWER_OFF));

}

void GpsGetData()
{
    GsmUartSendCommand(AT_GPS_GET_NAVI_DATA, sizeof(AT_GPS_GET_NAVI_DATA));
}
