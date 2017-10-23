/*
 * fifo.h
 *
 *  Created on: 10 gru 2015
 *      Author: Konrad
 */

#ifndef LIBRARIES_FIFO_H_
#define LIBRARIES_FIFO_H_

#include <app_fifo.h>
#include <stdint.h>

#define FIFO_LENGTH fifo_length(p_fifo)  /**< Macro for calculating the FIFO length. */

uint32_t    FifoLeftSpace(app_fifo_t * p_fifo);
void        FifoInit(app_fifo_t* fifo, uint8_t* buf, uint16_t buf_size);
void        FifoClear(app_fifo_t* fifo);
void        FifoGet(app_fifo_t* fifo, uint8_t* byte);
uint8_t     FifoPeek(app_fifo_t* fifo, uint16_t index);
void        FifoPut(app_fifo_t* fifo, uint8_t byte);
uint32_t    FifoIsEmpty(app_fifo_t* fifo);
#endif /* LIBRARIES_FIFO_H_ */
