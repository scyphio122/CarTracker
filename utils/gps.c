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
#include "RTC.h"
#include "parsing_utils.h"

gps_sample_t gpsLastSample;

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

static void GpsParseTime(char* hour, char* minute, char* second, char* millisecond, rtc_time_t* outTime)
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

void GpsStringifyCoord(gps_coord_t* coord, char* buf)
{
    //                     ddd*mm.mmmmN\0
    sprintf(buf, "%d*%d.%d%c", coord->degrees, coord->minutes, coord->seconds, coord->hemisphereDescriptor);
}

void GpsPowerOn()
{
    gsm_error_e err = GSM_OK;
    GsmUartSendCommand(AT_GPS_POWER_ON, sizeof(AT_GPS_POWER_ON), NULL);
    Rtc1DelayMs(10);
}

void GpsPowerOff()
{
    GsmUartSendCommand(AT_GPS_POWER_OFF, sizeof(AT_GPS_POWER_OFF), NULL);
}

static void GpsDisableEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_DISABLE, sizeof(AT_GPS_EPO_DISABLE), NULL);
}

static void GpsEnableEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_ENABLE, sizeof(AT_GPS_EPO_ENABLE), NULL);
}

static void GpsTriggerEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_EXECUTE, sizeof(AT_GPS_EPO_EXECUTE), NULL);
}

void GpsAgpsTrigger()
{
    char response[32];
    char* answer = NULL;

    memset(response, 0, sizeof(response));

    // Configure context 2 for PDP - EPO uses only context 2
    GsmUartSendCommand("AT+QIFGCNT=2", sizeof("AT+QIFGCNT=2"), NULL);
    // Configure APN
    GsmUartSendCommand("AT+QICSGP=1,\"internet\"", sizeof("AT+QICSGP=1,\"internet\""), NULL);

    do
    {
    // Check if GNSS has time synchronized with GSM
    GsmUartSendCommand("AT+QGNSSTS?", sizeof("AT+QGNSSTS?"), response);

    // Get the pointer to the answer number
    answer = strstr(response, " ");
    answer++;
    if (*answer == '1')
    {
        break;
    }

    Rtc1DelayMs(200);
    } while(*answer != 1);

    GpsEnableEPO();
    GpsTriggerEPO();
}

void GpsSetReferencePosition(gps_coord_t* latitude, gps_coord_t* longitude)
{
    char _latitude[16];
    char _longtitude[16];
    char cmd[64];

    memset(_latitude, 0, sizeof(_latitude));
    memset(_longtitude, 0, sizeof(_longtitude));
    memset(cmd, 0 , sizeof(cmd));

    sprintf(_latitude, "%d.%d%d", latitude->degrees, latitude->minutes, latitude->seconds);
    sprintf(_longtitude, "%d.%d%d", longitude->degrees, longitude->minutes, longitude->seconds);

    sprintf(cmd, "%s=%s,%s", AT_GPS_SET_REF_LOCATION, _latitude, _longtitude);

    GsmUartSendCommand(cmd, strlen(cmd) + 1, NULL);
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
            GpsParseMessageVTG(uartRxFifo.p_buf, uartRxFifo.buf_size_mask);
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

   rtc_time_t time;
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
       {
           if (*curField == ',')
           {
               curField--;
           }
           fieldNumber++;
           continue;
       }
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
           }break;

           case ALTITUDE:
           {
               gpsLastSample.altitude = (uint16_t)_atoi(curField, fieldSize);
           }break;

           case FIXED_VALUE_1:
           {
           }break;

           case GEOID_SEPARATION:
           {
           }break;

           case FIXED_VALUE_2:
           {
           }break;

           case DGPS_AGE:
           {
           }break;

           case DGPS_STATION_ID:
           {
           }break;

           case END_CHARACTER:
           {
           }break;

           default:
               break;
       }
       fieldNumber++;
   }
}

gps_error_e GpsParseMessageVTG(uint8_t* msgBuffer, uint16_t msgBufferSize)
{
    // Get message start
    char* msgStart = strstr(msgBuffer, "VTG");

    // If start was not found then there is no GGA packet in the message
    if (msgStart == NULL)
        return GPS_NO_MSG_E;

    // Give the pointer to the '$' character
    int calcChecksum = _GpsCalcChecsum(msgStart - 3);
    char* msgEnd = strstr(msgStart, "\r\n");
    int msgChecksum = strtol((msgEnd - 2), NULL, 16);

    if ((calcChecksum ^ msgChecksum) != 0)
        return GPS_CHECKSUM_ERROR_E;

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
        {
            if (*curField == ',')
                curField--;

            fieldNumber++;
            continue;
        }

        switch (fieldNumber)
        {
            case COURSE_OVER_GROUND_TRUE:
            {
                gpsLastSample.azimuth = _atoi(curField, fieldSize)*100;
                // Check existance of the dot
                char* dotIndex = strstr(curField, ".");
                if (dotIndex < msgEnd)
                {
                    gpsLastSample.azimuth += _atoi(dotIndex+1, 2);
                }

            }break;

            case FIXED_VALUE_T:
            {

            }break;

            case COURSE_OVER_GROUND_MAGNETIC:
            {

            }break;

            case FIXED_VALUE_M:
            {

            }break;

            case SPEED_KNOTS:
            {

            }break;

            case FIXED_VALUE_N:
            {

            }break;

            case SPEED_KM:
            {
                gpsLastSample.speed = _atoi(curField, fieldSize)*100;
                // Check existance of the dot
                char* dotIndex = strstr(curField, ".");
                if (dotIndex < msgEnd)
                {
                    gpsLastSample.speed += _atoi(dotIndex+1, 2);
                }
            }break;

            case FIXED_VALUE_K:
            {
                if (*curField != 'K')
                    while(1);

            }break;
            case POSITIONING_MODE:
            {

            }break;

            case VTG_END_CHARACTER:
            {

            }break;
        }
        fieldNumber++;
    }
}

