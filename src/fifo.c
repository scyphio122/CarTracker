/*
 * fifo.c
 *
 *  Created on: 10 gru 2015
 *      Author: Konrad
 */
#include "app_fifo.h"
#include "fifo.h"
#include "stdbool.h"

uint32_t FifoLeftSpace(app_fifo_t * p_fifo)
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
void FifoInit(app_fifo_t* fifo, uint8_t* buf, uint16_t buf_size)
{
    app_fifo_init(fifo, buf, buf_size);
}

/**
 * This function clear the fifo (it sets the read and write indices to zero)
 *
 * \param fifo - fifo to clear
 */
inline void FifoClear(app_fifo_t* fifo)
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
inline void FifoGet(app_fifo_t* fifo, uint8_t* byte)
{
    app_fifo_get(fifo, byte);
}

/**
 * \brief This function peeks the byte at index (cur read index - index)
 * @param fifo
 * @param byte
 */
inline uint8_t FifoPeek(app_fifo_t* fifo, uint16_t index)
{
    int diff = fifo->read_pos - index;
    if (diff < 0)
    {
        diff += fifo->buf_size_mask;
    }

    return fifo->p_buf[fifo->read_pos - index];
}

/**
 * \brief Put the byte in the fifo
 *
 * \param fifo - the FIFO where the byte is to be put
 * \param byte - the byte to be put
 */
inline void FifoPut(app_fifo_t* fifo, uint8_t byte)
{
    app_fifo_put(fifo, byte);
}

/**
 * \brief Checks whether fifo is empty
 *
 * \param fifo - fifo to check
 *
 * \return      true - if fifo empty
 *              false - if fifo contains some data
 */
inline uint32_t FifoIsEmpty(app_fifo_t* fifo)
{
    if(fifo->read_pos == fifo->write_pos)
        return true;

    return false;
}
