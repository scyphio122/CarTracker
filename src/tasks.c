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
#include "file_system.h"
#include "ble_central.h"

volatile bool       isTrackInProgress = false;
volatile bool       isAlarmActivated = true;
volatile bool       isAlarmTriggered = false;
volatile uint32_t   gpsSamplingPeriodMs = 10 * 1000;
volatile int8_t     gpsSamplingTaskId = -1;
volatile uint32_t   alarmTimeoutMs  = 60 * 1000;
volatile uint32_t   alarmSmsPeriodMs = 10 * 60 * 1000;
volatile int8_t     alarmTimeoutTaskId = -1;
volatile int8_t     alarmTaskId = -1;

void TaskScanForKeyTag()
{
    BleCentralScanStart();
}

void TaskGpsGetSample(void)
{
    memset(&gpsLastSample, 0, sizeof(gpsLastSample));
    GpsRequestMessage(GPS_MSG_GGA);
    GpsRequestMessage(GPS_MSG_VTG);
    GsmHttpSendSample(&gpsLastSample);
//    Mem_Org_Store_Sample();
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
    SchedulerCancelOperation(&alarmTimeoutTaskId);
    isAlarmTriggered = false;
}

void TaskAbortAlarm()
{
    SchedulerCancelOperation(&alarmTaskId);
    isAlarmTriggered = false;
    isAlarmActivated = true;
}

void TaskStartNewTrack()
{
    if (!isTrackInProgress)
    {
        TaskScanForKeyTag();
        Mem_Org_Track_Start_Storage();
        SchedulerAddOperation(TaskAlarmTimeout, alarmTimeoutMs, &alarmTimeoutTaskId, false);
        SchedulerAddOperation(TaskGpsGetSample, gpsSamplingPeriodMs, &gpsSamplingTaskId, true);
        GsmHttpSendStartTrack();
        isTrackInProgress = true;
    }
}

void TaskEndCurrentTrack()
{
    Mem_Org_Track_Stop_Storage();
    SchedulerCancelOperation(&gpsSamplingTaskId);
    GsmHttpEndTrack();
    isTrackInProgress = false;
}

