/*
 * scheduler.c
 *
 *  Created on: Oct 26, 2017
 *      Author: Konrad Traczyk
 */

#include "scheduler.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include <string.h>

#define SCHEDULER_BUFFER_SIZE       16
static scheduler_entry_t _scheduleBuffer[SCHEDULER_BUFFER_SIZE];

volatile uint32_t scheduler_current_time_ms;

scheduler_error_code_e SchedulerCheckOperations()
{
    for (uint8_t i=0; i< SCHEDULER_BUFFER_SIZE; ++i)
    {
        if (_scheduleBuffer[i].isInProgress == true && _scheduleBuffer[i].timeFromNowMs == scheduler_current_time_ms)
        {
            _scheduleBuffer[i].isTimedOut = true;
        }
    }

    return E_SCHEDULER_OK;
}

scheduler_error_code_e SchedulerAddOperation(void (*callback)(void), volatile uint32_t timeMsFromNow, volatile uint8_t* taskIndex, bool isCyclic)
{
    scheduler_entry_t entry;

    for (uint8_t i=0; i< SCHEDULER_BUFFER_SIZE; ++i)
    {
        if (_scheduleBuffer[i].isInProgress == false)
        {
            entry.isInProgress = true;
            entry.isTimedOut = false;
            entry.callback = callback;
            entry.timeFromNowMs = scheduler_current_time_ms + timeMsFromNow;
            entry.isCyclic = isCyclic;
            memcpy(&_scheduleBuffer[i], &entry, sizeof(scheduler_entry_t));
            *taskIndex = i;
            return E_SCHEDULER_OK;
        }
    }

    return E_SCHEDULER_NO_RESOURCES;
}

scheduler_error_code_e SchedulerCancelOperation(volatile uint8_t* taskIndex)
{
    _scheduleBuffer[*taskIndex].isInProgress = false;
    *taskIndex = -1;
}

scheduler_error_code_e ScheduleExecutePendingOperations()
{
    for (uint8_t i=0; i< SCHEDULER_BUFFER_SIZE; ++i)
    {
        if (_scheduleBuffer[i].isInProgress == true && _scheduleBuffer[i].isTimedOut == true)
        {
            _scheduleBuffer[i].callback();
            if (_scheduleBuffer[i].isCyclic == false)
            {
                _scheduleBuffer[i].isInProgress = false;
            }
        }
    }

    return E_SCHEDULER_OK;
}

