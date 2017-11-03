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

typedef struct
{
    void*               p_buf;
    uint16_t            buf_size_mask;
    uint8_t             word_size;
    volatile uint16_t   read_pos;
    volatile uint16_t   write_pos;
}fifo_t;

uint32_t        FifoLeftSpace(fifo_t * p_fifo);
void            FifoInit(fifo_t* fifo, void* buf, uint16_t buf_size, uint8_t word_size);
void            FifoClear(fifo_t* fifo);
int             FifoGet(fifo_t* fifo, void* data);
uint32_t        FifoPeek(fifo_t* fifo, uint16_t index);
void            FifoPut(fifo_t* fifo, uint32_t data);
uint32_t        FifoIsEmpty(fifo_t* fifo);
#endif /* LIBRARIES_FIFO_H_ */
