/*
 * SPI.c
 *
 *  Created on: Sep 3, 2017
 *      Author: root
 */

#include "SPI.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "settings.h"
#include "pinout.h"
#include "nrf_gpio.h"
#include <stdint-gcc.h>
#include <core_cm4.h>
#include <cmsis_gcc.h>

#define SPI_DUMMY_BYTE	(uint8_t)0xFF

static E_SPI_Type s_spi0_type;
static E_SPI_Type s_spi1_type;
static E_SPI_Type s_spi2_type;

volatile static uint16_t s_spi0_bytes_to_send;
volatile static uint16_t s_spi0_bytes_sent;
static uint8_t*			 s_spi0_write_buffer;

volatile static uint16_t s_spi1_bytes_to_send;
volatile static uint16_t s_spi1_bytes_sent;
static uint8_t*			 s_spi1_write_buffer;

volatile static uint16_t s_spi2_bytes_to_send;
volatile static uint16_t s_spi2_bytes_sent;
static uint8_t*			 s_spi2_write_buffer;

volatile static uint16_t s_spi0_bytes_to_read;
volatile static uint16_t s_spi0_bytes_received;
static uint8_t*			 s_spi0_read_buffer;

volatile static uint16_t s_spi1_bytes_to_read;
volatile static uint16_t s_spi1_bytes_received;
static uint8_t*			 s_spi1_read_buffer;

volatile static uint16_t s_spi2_bytes_to_read;
volatile static uint16_t s_spi2_bytes_received;
static uint8_t*			 s_spi2_read_buffer;

volatile static bool	 s_spi0_is_reading;
volatile static bool	 s_spi1_is_reading;
volatile static bool	 s_spi2_is_reading;

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler()
{
	NRF_SPI0->EVENTS_READY = 0;
	static uint8_t dummy;
	if (s_spi0_is_reading)
	{
		s_spi0_read_buffer[s_spi0_bytes_received++] = NRF_SPI0->RXD;

		if (s_spi0_bytes_received < s_spi0_bytes_to_read)
		{
			NRF_SPI0->TXD = SPI_DUMMY_BYTE;
		}
		else
		{
			s_spi0_is_reading = false;
			NRF_SPI0->INTENCLR = SPI_INTENCLR_READY_Msk;
		}
	}
	else
	{
        dummy = NRF_SPI0->RXD;
        s_spi0_bytes_sent++;
		if (s_spi0_bytes_sent < s_spi0_bytes_to_send)
		{
			NRF_SPI0->TXD = s_spi0_write_buffer[s_spi0_bytes_sent];
		}
	}
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler()
{
	NRF_SPI1->EVENTS_READY = 0;
	static uint8_t dummy;

	if (s_spi1_is_reading)
	{
		s_spi1_read_buffer[s_spi1_bytes_received++] = NRF_SPI1->RXD;

		if (s_spi1_bytes_received < s_spi1_bytes_to_read)
		{
			NRF_SPI1->TXD = SPI_DUMMY_BYTE;
		}
		else
		{
			s_spi1_is_reading = false;
			NRF_SPI1->INTENCLR = SPI_INTENCLR_READY_Msk;
		}
	}
	else
	{
        dummy = NRF_SPI1->RXD;
        s_spi1_bytes_sent++;
		if (s_spi1_bytes_sent < s_spi1_bytes_to_send)
		{
			NRF_SPI1->TXD = s_spi1_write_buffer[s_spi1_bytes_sent];
		}
	}
}

void SPIM2_SPIS2_SPI2_IRQHandler()
{
	NRF_SPI2->EVENTS_READY = 0;
	static uint8_t dummy;

	if (s_spi2_is_reading)
	{
		s_spi2_read_buffer[s_spi2_bytes_received++] = NRF_SPI2->RXD;

		if (s_spi2_bytes_received < s_spi2_bytes_to_read)
		{
			NRF_SPI2->TXD = SPI_DUMMY_BYTE;
		}
		else
		{
			s_spi2_is_reading = false;
			NRF_SPI2->INTENCLR = SPI_INTENCLR_READY_Msk;
		}
	}
	else
	{
        dummy = NRF_SPI2->RXD;
        ++s_spi2_bytes_sent;
		if (s_spi2_bytes_sent < s_spi2_bytes_to_send)
		{
			NRF_SPI2->TXD = s_spi2_write_buffer[s_spi2_bytes_sent];
		}
	}
}


void SpiConfig(NRF_SPI_Type* spi,
				E_SPI_Frequency frequency,
				uint8_t bytes_order,
				uint8_t sck_phase,
				uint8_t sck_polarity,
				uint8_t csPin)
{

	uint32_t ret = 0xFFFFFFFF;
	switch ((uint32_t)spi)
	{
		case (uint32_t)NRF_SPI0:
		{
			s_spi0_type = E_SPI_REGULAR;

			nrf_gpio_cfg_output(SPI0_SCK_PIN);
			nrf_gpio_cfg_output(SPI0_MOSI_PIN);
			nrf_gpio_cfg_output(csPin);
			nrf_gpio_cfg_input(SPI0_MISO_PIN, GPIO_PIN_CNF_PULL_Pullup);

			spi->PSEL.SCK  = SPI0_SCK_PIN;
			spi->PSEL.MISO = SPI0_MISO_PIN;
			spi->PSEL.MOSI = SPI0_MOSI_PIN;

			#if SOFTDEVICE_ENABLED
				ret = sd_nvic_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SPI0_PRIORITY);
				ret = sd_nvic_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
			#else
				NVIC_SetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn, SPI0_PRIORITY);
				NVIC_EnableIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
			#endif
		}break;

		case (uint32_t)NRF_SPI1:
		{
			s_spi1_type = E_SPI_REGULAR;

			nrf_gpio_cfg_output(SPI1_SCK_PIN);
			nrf_gpio_cfg_output(SPI1_MOSI_PIN);
			nrf_gpio_cfg_output(csPin);
			nrf_gpio_cfg_input(SPI1_MISO_PIN, GPIO_PIN_CNF_PULL_Pullup);

			spi->PSEL.SCK  = SPI1_SCK_PIN;
			spi->PSEL.MISO = SPI1_MISO_PIN;
			spi->PSEL.MOSI = SPI1_MOSI_PIN;

			#if SOFTDEVICE_ENABLED
				sd_nvic_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SPI1_PRIORITY);
				sd_nvic_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
			#else
				NVIC_SetPriority(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn, SPI1_PRIORITY);
				NVIC_EnableIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
			#endif
		}break;

		case (uint32_t)NRF_SPI2:
		{
			s_spi2_type = E_SPI_REGULAR;

			nrf_gpio_cfg_output(SPI2_SCK_PIN);
			nrf_gpio_cfg_output(SPI2_MOSI_PIN);
			nrf_gpio_cfg_output(csPin);
			nrf_gpio_cfg_input(SPI2_MISO_PIN, GPIO_PIN_CNF_PULL_Pullup);

			spi->PSEL.SCK  = SPI2_SCK_PIN;
			spi->PSEL.MISO = SPI2_MISO_PIN;
			spi->PSEL.MOSI = SPI2_MOSI_PIN;

			#if SOFTDEVICE_ENABLED
				sd_nvic_SetPriority(SPIM2_SPIS2_SPI2_IRQn, SPI2_PRIORITY);
				sd_nvic_EnableIRQ(SPIM2_SPIS2_SPI2_IRQn);
			#else
				NVIC_SetPriority(SPIM2_SPIS2_SPI2_IRQn, SPI2_PRIORITY);
				NVIC_EnableIRQ(SPIM2_SPIS2_SPI2_IRQn);
			#endif
		}break;
	}

	nrf_gpio_pin_set(csPin);

	spi->INTENSET = SPI_INTENSET_READY_Msk;
	spi->FREQUENCY = frequency;
	spi->CONFIG = (bytes_order << SPI_CONFIG_ORDER_Pos) | (sck_phase << SPI_CONFIG_CPHA_Pos) | (sck_polarity << SPI_CONFIG_CPOL_Pos);
}

void SpiEnable(NRF_SPI_Type* spi)
{
	spi->ENABLE = SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos;
}

void SpiDisable(NRF_SPI_Type* spi)
{
	spi->ENABLE = 0;
}

E_SPI_Errors SpiWrite(NRF_SPI_Type* spi, uint8_t* in_buf, uint16_t data_size)
{
	E_SPI_Errors error = E_SPI_SUCCESS;

	switch ((uint32_t)spi)
	{
		case (uint32_t)NRF_SPI0:
		{
			s_spi0_bytes_sent = 0;
			s_spi0_bytes_to_send = data_size;
			s_spi0_write_buffer = in_buf;
			s_spi0_is_reading = false;

			NRF_SPI0->INTENSET = SPI_INTENSET_READY_Msk;
			spi->TXD = in_buf[s_spi0_bytes_sent];
			if (data_size > 1)	// Write second byte due to double buffering of SPI TXD register
				spi->TXD = in_buf[++s_spi0_bytes_sent];

			while (s_spi0_bytes_sent < s_spi0_bytes_to_send)
			{
#if SOFTDEVICE_ENABLED
				sd_app_evt_wait();
#else
				__WFE();
#endif
			}
			NRF_SPI0->INTENCLR = SPI_INTENCLR_READY_Msk;
		}break;

		case (uint32_t)NRF_SPI1:
		{
			s_spi1_bytes_sent = 0;
			s_spi1_bytes_to_send = data_size;
			s_spi1_write_buffer = in_buf;
			s_spi1_is_reading = false;

			NRF_SPI1->INTENSET = SPI_INTENSET_READY_Msk;
			spi->TXD = in_buf[s_spi1_bytes_sent];
			if (data_size > 1)	// Write second byte due to double buffering of SPI TXD register
				spi->TXD = in_buf[++s_spi1_bytes_sent];


			while (s_spi1_bytes_sent < s_spi1_bytes_to_send)
			{
#if SOFTDEVICE_ENABLED
				sd_app_evt_wait();
#else
				__WFE();
#endif
			}
			NRF_SPI1->INTENCLR = SPI_INTENCLR_READY_Msk;
		}break;

		case (uint32_t)NRF_SPI2:
		{
			s_spi2_bytes_sent = 0;
			s_spi2_bytes_to_send = data_size;
			s_spi2_write_buffer = in_buf;
			s_spi2_is_reading = false;

			NRF_SPI2->INTENSET = SPI_INTENSET_READY_Msk;
			spi->TXD = in_buf[s_spi2_bytes_sent];
			if (data_size > 1)	// Write second byte due to double buffering of SPI TXD register
				spi->TXD = in_buf[++s_spi2_bytes_sent];

			while (s_spi2_bytes_sent < s_spi2_bytes_to_send)
			{
#if SOFTDEVICE_ENABLED
				sd_app_evt_wait();
#else
				__WFE();
#endif
			}
			NRF_SPI2->INTENCLR = SPI_INTENCLR_READY_Msk;
		}break;
	}

	return error;
}

E_SPI_Errors SpiRead(NRF_SPI_Type* spi, uint8_t* out_buf, uint16_t data_size)
{
	E_SPI_Errors error = E_SPI_SUCCESS;
	switch ((uint32_t)spi)
	{
		case (uint32_t)NRF_SPI0:
		{
			s_spi0_bytes_received = 0;
			s_spi0_bytes_to_read = data_size;
			s_spi0_is_reading = true;
			s_spi0_read_buffer = out_buf;

			NRF_SPI0->INTENSET = SPI_INTENSET_READY_Msk;
			NRF_SPI0->TXD = SPI_DUMMY_BYTE;
			while (s_spi0_is_reading)
			{
#if SOFTDEVICE_ENABLED
				sd_app_evt_wait();
#else
				__WFE();
#endif
			}
		}break;

		case (uint32_t)NRF_SPI1:
		{
			s_spi1_bytes_received = 0;
			s_spi1_bytes_to_read = data_size;
			s_spi1_is_reading = true;
			s_spi1_read_buffer = out_buf;

			NRF_SPI1->INTENSET = SPI_INTENSET_READY_Msk;
			NRF_SPI1->TXD = SPI_DUMMY_BYTE;
			while (s_spi1_is_reading)
			{
#if SOFTDEVICE_ENABLED
				sd_app_evt_wait();
#else
				__WFE();
#endif
			}
		}break;

		case (uint32_t)NRF_SPI2:
		{
			s_spi2_bytes_received = 0;
			s_spi2_bytes_to_read = data_size;
			s_spi2_is_reading = true;
			s_spi2_read_buffer = out_buf;

			NRF_SPI2->INTENSET = SPI_INTENSET_READY_Msk;
			NRF_SPI2->TXD = SPI_DUMMY_BYTE;
			while (s_spi2_is_reading)
			{
#if SOFTDEVICE_ENABLED
				sd_app_evt_wait();
#else
				__WFE();
#endif
			}
		}break;
	}
	return error;
}

void SpiCSAssert(uint8_t cs_pin)
{
    NRF_GPIO->OUTCLR = 1 << cs_pin;
}

void SpiCSDeassert(uint8_t cs_pin)
{
    NRF_GPIO->OUTSET = 1 << cs_pin;
}
