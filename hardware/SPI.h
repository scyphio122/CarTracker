/*
 * SPI.h
 *
 *  Created on: Sep 3, 2017
 *      Author: root
 */

#ifndef HARDWARE_SPI_H_
#define HARDWARE_SPI_H_

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include <stdint-gcc.h>
#include "settings.h"

#define SPI0_PRIORITY	APPLICATION_IRQ_LOW_PRIORITY
#define SPI1_PRIORITY	APPLICATION_IRQ_HIGH_PRIORITY
#define SPI2_PRIORITY	APPLICATION_IRQ_LOW_PRIORITY

typedef enum
{
	E_SPI_SUCCESS,
	E_SPI_TIMEOUT
}E_SPI_Errors;

typedef enum
{
	E_SPI_FREQUENCY_125_kHz = SPI_FREQUENCY_FREQUENCY_K125,
	E_SPI_FREQUENCY_250_kHz = SPI_FREQUENCY_FREQUENCY_K250,
	E_SPI_FREQUENCY_500_kHz = SPI_FREQUENCY_FREQUENCY_K500,
	E_SPI_FREQUENCY_1_MHz 	= SPI_FREQUENCY_FREQUENCY_M1,
	E_SPI_FREQUENCY_2_MHz 	= SPI_FREQUENCY_FREQUENCY_M2,
	E_SPI_FREQUENCY_4_MHz 	= SPI_FREQUENCY_FREQUENCY_M4,
	E_SPI_FREQUENCY_8MHz  	= SPI_FREQUENCY_FREQUENCY_M8
}E_SPI_Frequency;

typedef enum
{
	E_SPI_REGULAR,
	E_SPI_MASTER
}E_SPI_Type;

void SpiConfig(NRF_SPI_Type* spi,
				E_SPI_Frequency frequency,
				uint8_t bytes_order,
				uint8_t sck_phase,
				uint8_t sck_polarity,
                uint8_t csPin);

void SpiEnable(NRF_SPI_Type* spi);

void SpiDisable(NRF_SPI_Type* spi);

void SpiSwitchPolarityPhase(NRF_SPI_Type* spi, uint8_t cpol, uint8_t cpha);

E_SPI_Errors SpiWrite(NRF_SPI_Type* spi, uint8_t* in_buf, uint16_t data_size);

E_SPI_Errors SpiRead(NRF_SPI_Type* spi, uint8_t* out_buf, uint16_t data_size);

void SpiCSAssert(uint8_t cs_pin);

void SpiCSDeassert(uint8_t cs_pin);



#endif /* HARDWARE_SPI_H_ */
