/*
 * tasks.h
 *
 *  Created on: Oct 31, 2017
 *      Author: Konrad Traczyk
 */

#ifndef INC_TASKS_H_
#define INC_TASKS_H_

void TaskGpsGetSample();

void TaskAlarmSendLocation();

void TaskAlarmTimeout();

void TaskDeactivateAlarm();

void TaskAbortAlarm();

void TaskStartNewTrack();

void TaskEndCurrentTrack();

#endif /* INC_TASKS_H_ */
