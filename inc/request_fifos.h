/*
 * request_fifos.h
 *
 *  Created on: Oct 31, 2017
 *      Author: Konrad Traczyk
 */

#ifndef INC_REQUEST_FIFOS_H_
#define INC_REQUEST_FIFOS_H_

#include <stdint-gcc.h>
#include "ble_uart_service.h"

typedef enum
{
    E_SYSTEM_SEND_SMS_WITH_LOCATION,
}system_commands_e;

uint32_t BleUartTaskFifoInit();

uint32_t BleUartAddPendingTask(ble_uart_communication_commands_e command);

uint32_t BleUartServicePendingTasks();

uint32_t SystemAddPendingTask(system_commands_e command);

uint32_t SystemAddPendingTaskParameters(void* parameterAddress);

uint32_t SystemServicePendingTasks();


#endif /* INC_REQUEST_FIFOS_H_ */
