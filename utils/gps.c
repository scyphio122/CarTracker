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
#include "pinout.h"
#include "nrf_gpio.h"
#include <string.h>
#include <stdlib.h>

gsm_sample_t gpsLastSample;

static int _pow(int base, int exp)
    {
      if(exp < 0)
        return -1;

        int result = 1;
        while (exp)
        {
            if (exp & 1)
                result *= base;
            exp >>= 1;
            base *= base;
        }

        return result;
    }

static int _atoi(char* input, uint8_t size)
{
    int value = 0;
    uint32_t decimal = _pow(10, size-1);

    for (int i=0; i < size; ++i)
    {
        value += (*input - '0') * decimal;
        decimal /= 10;
        input++;
    }

    return value;
}

static int _GpsCalcChecsum(uint8_t* messageStart)
{
    uint16_t checksum = 0;

    if (*messageStart == '$')
        messageStart++;

    while (*messageStart != '*')
    {
        checksum ^= *messageStart;
        messageStart++;
    }

    return checksum;
}

static void GpsParseTime(char* hour, char* minute, char* second, char* millisecond, gps_time_t* outTime)
{
    outTime->hour = _atoi(hour, 2);
    outTime->minute = _atoi(minute, 2);
    outTime->second = _atoi(second, 2);
    outTime->millisecond = _atoi(millisecond, 3);
}

static void GpsParseLatitude(char* coordinate, gps_coord_t* outCoord)
{
    outCoord->degrees = _atoi(coordinate, 2);
    outCoord->minutes = _atoi(coordinate + 2, 2);
    outCoord->seconds = _atoi(coordinate + 5, 4);
}

static void GpsParseLongtitude(char* coordinate, gps_coord_t* outCoord)
{
    outCoord->degrees = _atoi(coordinate, 3);
    outCoord->minutes = _atoi(coordinate + 3, 2);
    outCoord->seconds = _atoi(coordinate + 6, 4);
}

void GpsPowerOn()
{
    gsm_error_e err = GSM_OK;
    nrf_gpio_cfg_input(GPS_ENABLE_PIN, NRF_GPIO_PIN_PULLUP);
    uint8_t state = nrf_gpio_pin_read(GPS_ENABLE_PIN);
    GsmUartSendCommand(AT_GPS_POWER_READ, sizeof(AT_GPS_POWER_READ), NULL);
    GsmUartSendCommand(AT_GPS_POWER_ON, sizeof(AT_GPS_POWER_ON), NULL);
    GsmUartSendCommand(AT_GPS_POWER_READ, sizeof(AT_GPS_POWER_READ), NULL);

    while (nrf_gpio_pin_read(GPS_ENABLE_PIN) == 0)
    {

    }
}

void GpsPowerOff()
{
    GsmUartSendCommand(AT_GPS_POWER_OFF, sizeof(AT_GPS_POWER_OFF), NULL);
}

void GpsDisableEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_DISABLE, sizeof(AT_GPS_EPO_DISABLE), NULL);
}

void GpsEnableEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_ENABLE, sizeof(AT_GPS_EPO_ENABLE), NULL);
}

void GpsGetData()
{
    GsmUartSendCommand(AT_GPS_GET_NAVI_DATA, sizeof(AT_GPS_GET_NAVI_DATA), NULL);
}

void GpsRequestMessage(gps_message_type_e msgType)
{
    switch (msgType)
    {
        case GPS_MSG_RMC:
        {
            GsmUartSendCommand(AT_GPS_GET_NAVI_MSG("RMC"), sizeof(AT_GPS_GET_NAVI_MSG("RMC")), NULL);
        }break;

        case GPS_MSG_VTG:
        {
            GsmUartSendCommand(AT_GPS_GET_NAVI_MSG("VTG"), sizeof(AT_GPS_GET_NAVI_MSG("VTG")), NULL);
        }break;

        case GPS_MSG_GSA:
        {
            GsmUartSendCommand(AT_GPS_GET_NAVI_MSG("GSA"), sizeof(AT_GPS_GET_NAVI_MSG("GSA")), NULL);
        }break;

        case GPS_MSG_GGA:
        {
            GsmUartSendCommand(AT_GPS_GET_NAVI_MSG("GGA"), sizeof(AT_GPS_GET_NAVI_MSG("GGA")), NULL);
            GpsParseMessageGGA(uartRxFifo.p_buf, uartRxFifo.buf_size_mask);
        }break;

        case GPS_MSG_GSV:
        {
            GsmUartSendCommand(AT_GPS_GET_NAVI_MSG("GSV"), sizeof(AT_GPS_GET_NAVI_MSG("GSV")), NULL);
        }break;

        case GPS_MSG_GLL:
        {
            GsmUartSendCommand(AT_GPS_GET_NAVI_MSG("GLL"), sizeof(AT_GPS_GET_NAVI_MSG("GLL")), NULL);
        }break;

        // By default - get all of the messages
        default:
            GpsGetData();
    }
}

gps_error_e GpsParseMessageGGA(uint8_t* msgBuffer, uint16_t msgBufferSize)
{
   // Get message start
   char* msgStart = strstr(msgBuffer, "GGA");

   // If start was not found then there is no GGA packet in the message
   if (msgStart == NULL)
       return GPS_NO_MSG_E;

   // Give the pointer to the '$' character
   int calcChecksum = _GpsCalcChecsum(msgStart - 3);
   char* msgEnd = strstr(msgStart, "\r\n");
   int msgChecksum = strtol((msgEnd - 2), NULL, 16);

   if ((calcChecksum ^ msgChecksum) != 0)
       return GPS_CHECKSUM_ERROR_E;

   gps_time_t time;
   uint8_t fieldNumber = 0;

   for (char* curField = msgStart; curField <= msgEnd; ++curField)
   {
       curField = strstr(curField, ",");

       if (curField == NULL)
       {
           return GPS_OK_E;
       }

       // Point just after the ',' character
       curField++;
       char* fieldEnd = strstr(curField, ",");
       uint8_t fieldSize = (uint8_t)(fieldEnd - curField);

       if (fieldSize == 0)
           continue;

       switch (fieldNumber)
       {
           case UTC_TIME:
           {
               GpsParseTime(curField, curField+2, curField+4, curField+7, &time);
           }break;

           case LATITUDE:
           {
               GpsParseLatitude(curField, &(gpsLastSample.latitude));
           }break;

           case N_S:
           {
               gpsLastSample.latitude.hemisphereDescriptor = *curField;
           }break;

           case LONGTITUDE:
           {
               GpsParseLongtitude(curField, &(gpsLastSample.longtitude));
           }break;

           case E_W:
           {
               gpsLastSample.longtitude.hemisphereDescriptor = *curField;
           }break;

           case FIX_STATUS:
           {
               gpsLastSample.fixStatus = *curField;
           }break;

           case NUMBER_OF_SV:
           {
               gpsLastSample.numOfSattelites = *curField;
           }break;

           case HDOP:
           {
               continue;
           }break;

           case ALTITUDE:
           {
               gpsLastSample.altitude = (uint16_t)_atoi(curField, fieldSize);
           }break;

           case FIXED_VALUE_1:
           {
               continue;
           }break;

           case GEOID_SEPARATION:
           {
               continue;
           }break;

           case FIXED_VALUE_2:
           {
               continue;
           }break;

           case DGPS_AGE:
           {
               continue;
           }break;

           case DGPS_STATION_ID:
           {
               continue;
           }break;

           case END_CHARACTER:
           {
               continue;
           }break;

           default:
               break;
       }
       fieldNumber++;
   }

}
