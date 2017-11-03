/*
 * fifo.c
 *
 *  Created on: 10 gru 2015
 *      Author: Konrad
 */
#include "fifo.h"
#include "stdbool.h"
#include <string.h>

uint32_t FifoLeftSpace(fifo_t * p_fifo)
{
  uint32_t tmp = p_fifo->read_pos;
  return p_fifo->write_pos - tmp;
}


/**
 * This function initializes the fifo - sets the buffer, it's size and initializes the read and write inices
 *
 * \param fifo      - the fifo to be configured
 * \param buf       - the buffer which will be attached to the fifo
 * \param buf_size  - the size of the buffer which will be attached to the fifo
 */
void FifoInit(fifo_t* fifo, void* buf, uint16_t buf_size, uint8_t word_size)
{
//    app_fifo_init(fifo, buf, buf_size);
    fifo->p_buf = buf;
    fifo->buf_size_mask = buf_size;
    fifo->word_size = word_size;
    fifo->read_pos = 0;
    fifo->write_pos = 0;
}

/**
 * This function clear the fifo (it sets the read and write indices to zero)
 *
 * \param fifo - fifo to clear
 */
inline void FifoClear(fifo_t* fifo)
{
    fifo->read_pos = 0;
    fifo->write_pos = 0;
}

/**
 * \brief This function retrieves the byte of data from the fifo
 *
 * \param fifo - the fifo from which the byte is to be extracted
 * \param byte - pointer to the single byte buffer
 */
inline int FifoGet(fifo_t* fifo, void* data)
{
    // If fifo empty
    if (fifo->read_pos == fifo->write_pos)
    {
        *(uint8_t*)data = 0;
        return -1;
    }
    else
    {
        memcpy((uint8_t*)data, &((uint8_t*)fifo->p_buf)[fifo->read_pos*fifo->word_size], fifo->word_size);
    }

    fifo->read_pos++;
    if (fifo->read_pos == fifo->buf_size_mask)
    {
        fifo->read_pos = 0;
    }

    return 0;
}

/**
 * \brief This function peeks the byte at index (cur read index - index)
 * @param fifo
 * @param byte
 */
inline uint32_t FifoPeek(fifo_t* fifo, uint16_t index)
{
    int diff = fifo->write_pos - index - 1;
    if (diff < 0)
    {
        diff += fifo->buf_size_mask;
    }

    return ((uint8_t*)fifo->p_buf)[diff*fifo->word_size];
}

/**
 * \brief Put the data in the fifo
 *
 * \param fifo - the FIFO where the byte is to be put
 * \param data - the data to be put
 */
inline void FifoPut(fifo_t* fifo, uint32_t data)
{
    memcpy(&(((uint8_t*)fifo->p_buf)[fifo->write_pos*fifo->word_size]), &data, fifo->word_size);
    fifo->write_pos++;
    if (fifo->write_pos > fifo->buf_size_mask)
        fifo->write_pos = 0;

    ((uint8_t*)fifo->p_buf)[fifo->write_pos] = 0;
}

/**
 * \brief Checks whether fifo is empty
 *
 * \param fifo - fifo to check
 *
 * \return      true - if fifo empty
 *              false - if fifo contains some data
 */
inline uint32_t FifoIsEmpty(fifo_t* fifo)
{
    if(fifo->read_pos == fifo->write_pos)
        return true;

    return false;
}
