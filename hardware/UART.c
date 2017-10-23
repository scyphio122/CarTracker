/*
 * UART.c
 *
 *  Created on: Jun 14, 2017
 *      Author: root
 */
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_soc.h"
#include "nrf_nvic.h"
#include "UART.h"
#include "settings.h"
#include "pinout.h"
#include "nrf_gpio.h"
#include "nrf_clock.h"
#include "core_cm4.h"
#include "fifo.h"
#include <app_fifo.h>

static uint32_t 				s_uartBytesToSend;
static volatile uint32_t 		s_uartBytesSent;
static volatile bool			s_uartMessageSentFlag;
static uint8_t* 				s_uartTXBufferPtr;

static uint32_t 				s_uartBytesToRead;
static volatile uint32_t 		s_uartBytesRead;
static volatile bool			s_uartDataReadFinishedFlag;
static uint8_t* 				s_uartRXBufferPtr;
static volatile bool			s_uartReadAsyncCurUsed = 0;
static volatile uint8_t			s_uartReadEndCharacter;
static volatile bool            s_uartIsReading = false;
static volatile bool 			s_uartIsReadEndCharacterUsed;

static volatile e_uart_error 	s_uartErrno;

static uint8_t                  s_uartRxFifoBuf[1024];
static app_fifo_t               s_uartRxFifo;

void UARTE0_UART0_IRQHandler()
{
	if(NRF_UART0->EVENTS_RXDRDY)
	{
		NRF_UART0->EVENTS_RXDRDY = 0;
		static uint8_t rxChar;
		rxChar = NRF_UART0->RXD;

		if (!s_uartIsReading)
		{
		    FifoPut(&s_uartRxFifo, rxChar);
		}
		else
		{
            s_uartRXBufferPtr[s_uartBytesRead++] = rxChar;

            if((s_uartIsReadEndCharacterUsed && rxChar == s_uartReadEndCharacter) || s_uartBytesRead == s_uartBytesToRead)
            {
                NRF_UART0->INTENCLR = UART_INTENCLR_RXDRDY_Msk;
                s_uartDataReadFinishedFlag = true;

                ///	If asynchroneous read has ended clear the flag to inform main context about this
                if(s_uartReadAsyncCurUsed)
                    s_uartReadAsyncCurUsed = false;
            }
		}
	}

	if(NRF_UART0->EVENTS_TXDRDY)
	{
		NRF_UART0->EVENTS_TXDRDY = 0;
		if(s_uartBytesSent < s_uartBytesToSend)
		{
			NRF_UART0->TXD = s_uartTXBufferPtr[s_uartBytesSent++];
		}
		else
		{
			s_uartMessageSentFlag = true;
			NRF_UART0->INTENCLR = UART_INTENCLR_TXDRDY_Msk;
		}

	}
}

e_uart_error UartReadDataNumberSync(uint8_t* dataBuffer, uint32_t dataSize)
{
	///	Check if currently async uart read is not in use - if is, wait till it ends
	while(s_uartReadAsyncCurUsed)
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
	}
	s_uartErrno = UART_NO_ERROR;

	s_uartBytesRead = 0;
	s_uartBytesToRead = dataSize;
	s_uartRXBufferPtr = dataBuffer;
	s_uartDataReadFinishedFlag = false;
	s_uartReadAsyncCurUsed = false;
	s_uartIsReadEndCharacterUsed = false;
	s_uartIsReading = true;

	NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Msk;
	NRF_UART0->TASKS_STARTRX = 1;

	while(!s_uartDataReadFinishedFlag)
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
	}

    NRF_UART0->TASKS_STOPRX = 1;
    s_uartIsReading = false;


	return s_uartErrno;
}

e_uart_error UartReadDataWithPatternSync(uint8_t* dataBuffer, uint8_t* endWord, uint8_t endWordSize)
{
	///	Check if currently async uart read is not in use - if is, wait till it ends
	while(s_uartReadAsyncCurUsed)
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
	}
	s_uartErrno = UART_NO_ERROR;

	s_uartBytesRead = 0;
	s_uartBytesToRead = 0;
	s_uartRXBufferPtr = dataBuffer;
	s_uartDataReadFinishedFlag = false;
	s_uartReadAsyncCurUsed = false;
	s_uartIsReadEndCharacterUsed = true;
    s_uartIsReading = true;

	NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Msk;
	NRF_UART0->TASKS_STARTRX = 1;

	bool dataReceiving = true;
	uint16_t matchingCount = 0;
	while (dataReceiving)
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
		for (uint8_t i=0; i<endWordSize; ++i)
		{
		    if (s_uartBytesRead >= (endWordSize - 1))
		    {
                if (endWord[i] == s_uartRXBufferPtr[s_uartBytesRead - endWordSize - i - 1])
                {
                    matchingCount++;
                }

                if (matchingCount == endWordSize)
                {
                    dataReceiving = false;
                }
		    }
		}
	}

	NRF_UART0->TASKS_STOPRX = 1;
    s_uartIsReading = false;

	return s_uartErrno;
}

e_uart_error UartSendDataSync(uint8_t* dataToSend, uint32_t dataSize)
{
	s_uartBytesToSend = dataSize;
	s_uartBytesSent = 0;
	s_uartTXBufferPtr = dataToSend;
	s_uartMessageSentFlag = false;
	s_uartErrno = UART_NO_ERROR;
    s_uartIsReading = false;

	NRF_UART0->INTENSET = UART_INTENSET_TXDRDY_Msk;
	NRF_UART0->TASKS_STARTTX = 1;
	NRF_UART0->TXD = s_uartTXBufferPtr[s_uartBytesSent++];
	//NVIC_SetPendingIRQ(UARTE0_UART0_IRQn);


//	while(s_uartBytesSent < s_uartBytesToSend)
//	{
//		while(NRF_UART0->EVENTS_TXDRDY == 0)
//		{}
//		NRF_UART0->TXD = s_uartTXBufferPtr[s_uartBytesSent++];
//	}

	while(!s_uartMessageSentFlag)
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif

	}

	NRF_UART0->TASKS_STOPTX = 1;
	return s_uartErrno;
}


void UartEnable()
{
	NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Msk;
}

void UartDisable()
{
	NRF_UART0->ENABLE = 0;
}

/**
 * @brief This function configures the UART0 peripheral.
 *
 * @param baudrateBitfield - possible values:
 * 		UART_BAUDRATE_BAUDRATE_Baud1200
 *		UART_BAUDRATE_BAUDRATE_Baud2400
 *		UART_BAUDRATE_BAUDRATE_Baud4800
 *		UART_BAUDRATE_BAUDRATE_Baud9600
 *		UART_BAUDRATE_BAUDRATE_Baud14400
 *		UART_BAUDRATE_BAUDRATE_Baud19200
 *		UART_BAUDRATE_BAUDRATE_Baud28800
 *		UART_BAUDRATE_BAUDRATE_Baud31250
 *	 	UART_BAUDRATE_BAUDRATE_Baud38400
 *		UART_BAUDRATE_BAUDRATE_Baud56000
 *		UART_BAUDRATE_BAUDRATE_Baud57600
 *		UART_BAUDRATE_BAUDRATE_Baud76800
 *		UART_BAUDRATE_BAUDRATE_Baud115200
 *		UART_BAUDRATE_BAUDRATE_Baud230400
 *		UART_BAUDRATE_BAUDRATE_Baud250000
 *	 	UART_BAUDRATE_BAUDRATE_Baud460800
 *		UART_BAUDRATE_BAUDRATE_Baud921600
 *		UART_BAUDRATE_BAUDRATE_Baud1M
 *
 *	@param parityBitfield
 *		UART_CONFIG_PARITY_Excluded
 *		UART_CONFIG_PARITY_Included
 *
 *	@param hardwareFlowControl
 *		UART_CONFIG_HWFC_Disabled
 *		UART_CONFIG_HWFC_Enabled
 */
void UartConfig(uint32_t baudrateBitfield, uint32_t parity, uint32_t hardwareFlowControl)
{
#if SOFTDEVICE_ENABLED
	uint32_t retval = 0xFFFFFFFF;
	retval = sd_nvic_SetPriority(UARTE0_UART0_IRQn, APPLICATION_IRQ_LOW_PRIORITY);
	retval = sd_nvic_EnableIRQ(UARTE0_UART0_IRQn);
	retval = sd_nvic_ClearPendingIRQ(UARTE0_UART0_IRQn);
#else
	NVIC_SetPriority(UARTE0_UART0_IRQn, APPLICATION_IRQ_LOW_PRIORITY);
	NVIC_EnableIRQ(UARTE0_UART0_IRQn);
#endif

	nrf_gpio_cfg_output(UART_TX_PIN);
	nrf_gpio_pin_set(UART_TX_PIN);
	nrf_gpio_cfg_input(UART_RX_PIN, NRF_GPIO_PIN_PULLUP);

	NRF_UART0->BAUDRATE = baudrateBitfield;
	NRF_UART0->CONFIG = (parity << UART_CONFIG_PARITY_Pos) | (hardwareFlowControl << UART_CONFIG_HWFC_Pos);

	NRF_UART0->PSELRXD = UART_RX_PIN;
	NRF_UART0->PSELTXD = UART_TX_PIN;

//	NRF_UART0->INTENSET = UART_INTENSET_ERROR_Msk;

	if(hardwareFlowControl)
	{
		nrf_gpio_cfg_output(UART_CTS_PIN);
		nrf_gpio_cfg_input(UART_RTS_PIN, NRF_GPIO_PIN_PULLUP);

		NRF_UART0->PSELCTS = UART_CTS_PIN;
		NRF_UART0->PSELRTS = UART_RTS_PIN;
	}

	FifoInit(&s_uartRxFifo, s_uartRxFifoBuf, sizeof(s_uartRxFifoBuf));
}
