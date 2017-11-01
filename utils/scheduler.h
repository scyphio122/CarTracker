/*
 * scheduler.h
 *
 *  Created on: Oct 26, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_SCHEDULER_H_
#define UTILS_SCHEDULER_H_

#include <stdint-gcc.h>
#include <stdbool.h>

typedef enum
{
    E_SCHEDULER_OK,
    E_SCHEDULER_NO_OPERATION_TIMED_OUT,
    E_SCHEDULER_NO_RESOURCES
}scheduler_error_code_e;

typedef struct
{
    uint32_t    timeFromNowMs;
    void        (*callback)(void);
    bool        isTimedOut;
    bool        isInProgress;
    bool        isCyclic;
}scheduler_entry_t;

extern volatile uint32_t scheduler_current_time_ms;

scheduler_error_code_e SchedulerCheckOperations();
scheduler_error_code_e SchedulerAddOperation(void (*callback)(), volatile uint32_t timeMsFromNow, volatile uint8_t* taskIndex, bool isCyclic);
scheduler_error_code_e SchedulerCancelOperation(uint8_t taskIndex);
scheduler_error_code_e ScheduleExecutePendingOperations();
#endif /* UTILS_SCHEDULER_H_ */
