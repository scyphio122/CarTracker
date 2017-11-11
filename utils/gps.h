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

typedef enum
{
    COURSE_OVER_GROUND_TRUE,
    FIXED_VALUE_T,
    COURSE_OVER_GROUND_MAGNETIC,
    FIXED_VALUE_M,
    SPEED_KNOTS,
    FIXED_VALUE_N,
    SPEED_KM,
    FIXED_VALUE_K,
    POSITIONING_MODE,
    VTG_END_CHARACTER
}gps_vtg_message_fields_e;

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
    int16_t             acceleration;
    uint32_t            timestamp;

}gps_sample_t;

extern gps_sample_t gpsLastSample;

void GpsPowerOn();

void GpsPowerOff();

void GpsSetReferencePosition(gps_coord_t* latitude, gps_coord_t* longitude);

void GpsAgpsTrigger();

void GpsGetData();

void GpsStringifyCoord(gps_coord_t* coord, char* buf);

void GpsRequestMessage(gps_message_type_e msgType);

gps_error_e GpsParseMessageGGA(uint8_t* msgBuffer, uint16_t msgBufferSize);

gps_error_e GpsParseMessageVTG(uint8_t* msgBuffer, uint16_t msgBufferSize);
#endif /* UTILS_GPS_H_ */
