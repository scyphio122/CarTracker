/*
 * tasks.c
 *
 *  Created on: Oct 31, 2017
 *      Author: Konrad Traczyk
 */

#include "tasks.h"
#include "gps.h"
#include "gsm.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include <stdio.h>
#include "scheduler.h"
#include <string.h>
#include "parsing_utils.h"

volatile bool       isTrackInProgress = false;
volatile bool       isAlarmActivated = true;
volatile bool       isAlarmTriggered = false;
volatile uint32_t   alarmSmsPeriodMs = 10 * 60 * 1000;
volatile int8_t     alarmTimeoutTaskId = -1;
volatile int8_t     alarmTaskId = -1;

void TaskGpsGetSample(void)
{
    memset(&gpsLastSample, 0, sizeof(gpsLastSample));
    GpsRequestMessage(GPS_MSG_GGA);
    GpsRequestMessage(GPS_MSG_VTG);
    GsmHttpSendSample(&gpsLastSample);
}

void TaskAlarmSendLocation()
{
    char telNum[12];
//        sprintf(telNum, "%llu", gsmOwnerDeviceNumber);
    _itoa(gsmOwnerDeviceNumber, telNum, sizeof(telNum));
    char localization[64];
    memset(localization, 0, sizeof(localization));
    if (gpsLastSample.fixStatus == GPS_FIX_NO_FIX)
    {
        memcpy(localization, "No fix", sizeof("No fix"));
    }
    else
    {
        sprintf(localization, "Latitude: %d*%d.%d'%c;Longitutde: %d*%d.%d'%c",
                gpsLastSample.latitude.degrees,
                gpsLastSample.latitude.minutes,
                gpsLastSample.latitude.seconds,
                gpsLastSample.latitude.hemisphereDescriptor,
                gpsLastSample.longtitude.degrees,
                gpsLastSample.longtitude.minutes,
                gpsLastSample.longtitude.seconds,
                gpsLastSample.longtitude.hemisphereDescriptor);
    }

    GsmSmsSend(telNum, localization);
}

void TaskAlarmTimeout()
{
    if (isAlarmActivated)
    {
        TaskAlarmSendLocation();
        SchedulerAddOperation(TaskAlarmSendLocation, alarmSmsPeriodMs, &alarmTaskId, true);
        isAlarmTriggered = true;
    }
}

void TaskDeactivateAlarm()
{
    SchedulerCancelOperation(alarmTimeoutTaskId);
    isAlarmTriggered = false;
}

void TaskAbortAlarm()
{
    SchedulerCancelOperation(alarmTaskId);
    alarmTaskId = -1;
    isAlarmTriggered = false;
    isAlarmActivated = true;
}

void TaskStartNewTrack()
{

}

void TaskEndCurrentTrack()
{

}

