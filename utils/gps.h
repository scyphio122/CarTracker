/*
 * gps.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_GPS_H_
#define UTILS_GPS_H_

#include <stdint-gcc.h>

typedef enum
{
    GPS_OK_E,
    GPS_NO_MSG_E,
    GPS_CHECKSUM_ERROR_E
}gps_error_e;

typedef enum
{
    GPS_MSG_RMC,
    GPS_MSG_VTG,
    GPS_MSG_GGA,
    GPS_MSG_GSA,
    GPS_MSG_GSV,
    GPS_MSG_GLL
}gps_message_type_e;

typedef enum
{
    UTC_TIME,
    LATITUDE,
    N_S,
    LONGTITUDE,
    E_W,
    FIX_STATUS,
    NUMBER_OF_SV,
    HDOP,
    ALTITUDE,
    FIXED_VALUE_1,
    GEOID_SEPARATION,
    FIXED_VALUE_2,
    DGPS_AGE,
    DGPS_STATION_ID,
    END_CHARACTER

}gps_gga_message_fields_e;

typedef struct
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
}gps_time_t;

typedef struct
{
    uint8_t  degrees;
    uint8_t  minutes;
    uint16_t seconds;
    char     hemisphereDescriptor;
}gps_coord_t;

typedef enum
{
    GPS_FIX_NO_FIX = '0',
    GPS_FIX_GNSS_FIX = '1',
    GPS_FIX_DGPS_FIX = '2',
    GPS_FIX_ESTIMATED = '6'
}gps_fix_status_e;

typedef struct
{
    gps_fix_status_e    fixStatus;
    uint8_t             numOfSattelites;
    gps_coord_t         latitude;
    gps_coord_t         longtitude;
    uint16_t            altitude;
    uint16_t            azimuth;
    uint16_t            speed;
    uint16_t            acceleration;
    uint32_t            timestamp;

}gsm_sample_t;

void GpsPowerOn();

void GpsPowerOff();

void GpsDisableEPO();

void GpsEnableEPO();

void GpsGetData();

void GpsRequestMessage(gps_message_type_e msgType);

gps_error_e GpsParseMessageGGA(uint8_t* msgBuffer, uint16_t msgBufferSize);


#endif /* UTILS_GPS_H_ */
