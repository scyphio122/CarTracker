/*
 * parsing_utils.c
 *
 *  Created on: Oct 30, 2017
 *      Author: Konrad Traczyk
 */

#include "parsing_utils.h"
#include <stdint-gcc.h>

uint64_t _pow(int base, int exp)
{
  if(exp < 0)
    return -1;

    uint64_t result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}

int64_t _atoi(char* input, uint8_t size)
{
    int64_t value = 0;
    uint64_t decimal = _pow(10, size-1);

    for (int i=0; i < size; ++i)
    {
        value += (*input - '0') * decimal;
        decimal /= 10;
        input++;
        if (*input == '.')
            break;
    }

    return value;
}

void _itoa(uint64_t value, char* buffer, uint16_t bufferSize)
{
    int range = 0;
    uint64_t temp = value;
    int64_t mask = 10;

    do
    {
        range++;
        temp = temp/10;
    }while(temp != 0);

    if (range > bufferSize)
        return;

    buffer[range--] = '\0';

    do
    {
        buffer[range--] = ((value % mask)/ (mask/10)) + '0';

        mask *= 10;
    }while (range >= 0);
}