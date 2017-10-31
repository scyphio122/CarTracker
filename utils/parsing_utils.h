/*
 * parsing_utils.h
 *
 *  Created on: Oct 30, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_PARSING_UTILS_H_
#define UTILS_PARSING_UTILS_H_

#include <stdint-gcc.h>

uint64_t _pow(int base, int exp);

int64_t _atoi(char* input, uint8_t size);

void _itoa(uint64_t value, char* buffer, uint16_t bufferSize);


#endif /* UTILS_PARSING_UTILS_H_ */
