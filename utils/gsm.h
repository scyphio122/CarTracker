/*
 * gsm.h
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_GSM_H_
#define UTILS_GSM_H_

void GsmInit();

void GsmPowerOn();

void GsmPowerOff();

void GsmUartSendCommand(void* command, uint16_t commandSize);

#endif /* UTILS_GSM_H_ */
