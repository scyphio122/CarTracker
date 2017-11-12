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
#include "lsm6dsm.h"
#include "pinout.h"
#include "nrf_gpio.h"

volatile int8_t     imuMovementCheckTaskId = -1;
volatile int8_t     gpsSamplingTaskId = -1;
volatile int8_t     alarmTimeoutTaskId = -1;
volatile int8_t     alarmTaskId = -1;

volatile bool       isTrackInProgress = false;
volatile bool       isAlarmActivated = true;
volatile bool       isAlarmTriggered = false;
volatile uint32_t   gpsSamplingPeriodMs = 10 * 1000;
volatile uint32_t   alarmTimeoutMs  = 60 * 1000;
volatile uint32_t   alarmSmsPeriodMs = 10 * 60 * 1000;

volatile uint8_t    gpsStopSamplesCount = 0;

void TaskStartCarMovementDetection()
{
    SchedulerAddOperation(TaskCarMovementDetectionCheck, 5000, &imuMovementCheckTaskId, true);
}

void TaskCarMovementDetectionCheck()
{
    // If the car movement was detected, then schedule the start new track task as soon as possible
    if (ImuIsWakeUpIRQ())
    {
        nrf_gpio_pin_clear(DEBUG_RED_LED_PIN);
        SchedulerCancelOperation(&imuMovementCheckTaskId);
        SchedulerAddOperation(TaskStartNewTrack, 0, NULL, false);
    }
}

void TaskStartNewTrack()
{
    if (!isTrackInProgress)
    {
        gpsStopSamplesCount = 0;

        GpsPowerOn();
        ImuDisableWakeUpIRQ();
//        TaskScanForKeyTag();
//        Mem_Org_Track_Start_Storage();
        SubtaskStartTrackAssessment();
//        SchedulerAddOperation(TaskAlarmTimeout, alarmTimeoutMs, &alarmTimeoutTaskId, false);
        SchedulerAddOperation(TaskGpsGetSample, gpsSamplingPeriodMs, &gpsSamplingTaskId, true);
//        GsmHttpSendStartTrack();
        isTrackInProgress = true;
    }
}

void TaskScanForKeyTag()
{
    BleCentralScanStart();
}

/**
 * @brief This task is triggered when during the time window after vehicle movement detection the Main board did not receive correct deactivating command
 *          from the Key Tag. It triggers Task Alarm send location - which continuously sends sms with vehicle location every cycle
 *          set by \ref alarmSmsPeriod (default 10 mins). It can be stopped by sending SMS from the owner phone
 */
void TaskAlarmTimeout()
{
    if (isAlarmActivated)
    {
        TaskAlarmSendLocation();
        SchedulerAddOperation(TaskAlarmSendLocation, alarmSmsPeriodMs, &alarmTaskId, true);
        isAlarmTriggered = true;
    }
}

/**
 * @brief This task is triggered when the Main Board received correct deactivating command from the Key Tag
 */
void TaskDeactivateAlarm()
{
    SchedulerCancelOperation(&alarmTimeoutTaskId);
    isAlarmTriggered = false;
}

/**
 * @brief This task should be triggered via SMS from the user to clear false alarms
 */
void TaskAbortAlarm()
{
    SchedulerCancelOperation(&alarmTaskId);
    isAlarmTriggered = false;
    isAlarmActivated = true;
}

/**
 * @brief This task collects every \ref gpsSamplingPeriodMs a sample with vehicle's GPS localization
 */
void TaskGpsGetSample(void)
{
    memset(&gpsLastSample, 0, sizeof(gpsLastSample));
    GpsRequestMessage(GPS_MSG_GGA);
    GpsRequestMessage(GPS_MSG_VTG);

    if (gpsLastSample.fixStatus != GPS_FIX_NO_FIX &&
            gpsLastSample.fixStatus != 0)
    {
        nrf_gpio_pin_set(DEBUG_RED_LED_PIN);
        nrf_gpio_pin_clear(DEBUG_ORANGE_LED_PIN);
    }

    if (gpsLastSample.speed < 150)
    {
        gpsStopSamplesCount++;
    }

    if (gpsStopSamplesCount >= 3)
    {
        TaskEndCurrentTrack();
        return;
    }

//    gpsLastSample.acceleration = SubtaskGetAcceleration();

    GsmHttpSendSample(&gpsLastSample);
//    Mem_Org_Store_Sample();
}

/**
 * @brief This task sends every \ref alarmSmsPeriodMs an SMS with vehicle's location on the user phone
 */
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

void SubtaskStartTrackAssessment()
{
    ImuFifoStart();
}

int16_t SubtaskGetAcceleration()
{
    ImuFifoGetAllSamples(NULL, 0);
    int16_t accVal = (int16_t)ImuGetMeanResultantAccelerationValueFromReadSamples();
    ImuFifoFlush();
    return accVal;
}

void SubtaskStopTrackAssessment()
{
    ImuFifoStop();
}

void TaskEndCurrentTrack()
{
//    Mem_Org_Track_Stop_Storage();
    GsmPowerOff();
    SchedulerCancelOperation(&gpsSamplingTaskId);
    SubtaskStopTrackAssessment();
    ImuEnableWakeUpIRQ();

    GsmHttpEndTrack();

    isTrackInProgress = false;

    nrf_gpio_pin_set(DEBUG_RED_LED_PIN);
    nrf_gpio_pin_set(DEBUG_ORANGE_LED_PIN);

}

