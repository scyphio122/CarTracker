/*
 * tasks.h
 *
 *  Created on: Oct 31, 2017
 *      Author: Konrad Traczyk
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

#include <stdint-gcc.h>

void TaskStartCarMovementDetection();

void TaskCarMovementDetectionCheck();

void TaskStartNewTrack();

void TaskDeactivateAlarm();

void TaskAbortAlarm();

void TaskGpsGetSample();

void TaskScanForKeyTag();

void TaskAlarmSendLocation();

void TaskAlarmTimeout();

void TaskEndCurrentTrack();

void SubtaskStartTrackAssessment();

int16_t SubtaskGetAcceleration();

void SubtaskStopTrackAssessment();

#endif /* INC_TASKS_H_ */
