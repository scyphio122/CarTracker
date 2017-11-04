/*
 * RTC.c
 *
 *  Created on: Sep 9, 2017
 *      Author: root
 */

#include "RTC.h"
#include "nrf52.h"
#include "core_cm4.h"
#include "nrf_nvic.h"
#include "stdlib.h"
#include <stdint-gcc.h>
#include <time.h>

#define DELAY_REGISTER_MAIN			0
#define	TIMEOUT_REGISTER_MAIN 		1
#define	TIMEOUT_REGISTER_SECOND		2
#define TIMEOUT_REGISTER_THIRD  	3

volatile rtc_timeout_t rtcTimeoutArray[RTC_TIMEOUT_ARRAY_SIZE];		/**< Array in which active timeout requests are held */
volatile bool rtc1_delay_completed_flag;							/**< Delay flag - set to true if the delay has been completed */
volatile bool rtc2_delay_completed_flag;							/**< Delay flag - set to true if the delay has been completed */
volatile bool rtc2_timeout_triggered_flag;							/**< Timeout flag - set to true if the timeout has been triggered */

uint32_t rtc_timestamp_partial;

#if !SOFTDEVICE_ENABLED
void RTC0_IRQHandler()
{

}
#endif

void RTC1_IRQHandler()
{
	if (NRF_RTC1->EVENTS_COMPARE[DELAY_REGISTER_MAIN])			/*< RTC DELAY */
	{
		NRF_RTC1->EVENTS_COMPARE[DELAY_REGISTER_MAIN] = 0;
		rtc1_delay_completed_flag = true;
	}

	if (NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN])		/*< RTC TIMEOUT */
	{
		NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN] = 0;
		RTCDisableComparingReg(NRF_RTC1, TIMEOUT_REGISTER_MAIN);
		rtcTimeoutArray[TIMEOUT_REGISTER_MAIN - 1].timeoutTriggeredFlag = true;
	}

	if (NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_SECOND])		/*< RTC TIMEOUT SECOND */
	{
		NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_SECOND] = 0;
		RTCDisableComparingReg(NRF_RTC1, TIMEOUT_REGISTER_SECOND);
		rtcTimeoutArray[TIMEOUT_REGISTER_SECOND - 1].timeoutTriggeredFlag = true;
	}

	if (NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_THIRD])		/*< RTC TIMEOUT THIRD */
	{
		NRF_RTC1->EVENTS_COMPARE[TIMEOUT_REGISTER_THIRD] = 0;
		RTCDisableComparingReg(NRF_RTC1, TIMEOUT_REGISTER_THIRD);
		rtcTimeoutArray[TIMEOUT_REGISTER_THIRD - 1].timeoutTriggeredFlag = true;
	}

	if (NRF_RTC1->EVENTS_OVRFLW)
	{
	    NRF_RTC1->EVENTS_OVRFLW = 0;
	    rtc_timestamp_partial += 0x00FFFFFF/RTC0_FREQUENCY;
	}
}

void RTC2_IRQHandler()
{
	if (NRF_RTC2->EVENTS_COMPARE[DELAY_REGISTER_MAIN])			/*< RTC DELAY */
	{
		NRF_RTC2->EVENTS_COMPARE[DELAY_REGISTER_MAIN] = 0;
		rtc2_delay_completed_flag = true;
	}
	else
	if (NRF_RTC2->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN])		/*< RTC TIMEOUT */
	{
		NRF_RTC2->EVENTS_COMPARE[TIMEOUT_REGISTER_MAIN] = 0;
		rtc2_timeout_triggered_flag = true;
	}
	else
	if (NRF_RTC2->EVENTS_COMPARE[2])
	{
		NRF_RTC2->EVENTS_COMPARE[2] = 0;
	}
	else
	if (NRF_RTC2->EVENTS_COMPARE[3])
	{
		NRF_RTC2->EVENTS_COMPARE[3] = 0;
	}

}

RTC_Error_e RTCDelay(NRF_RTC_Type* RTC, uint32_t time_ticks)
{
	volatile bool* delayFlag = NULL;

	if (RTC == NRF_RTC1)
	{
		rtc1_delay_completed_flag = false;
		delayFlag = &rtc1_delay_completed_flag;
	}
	else
	if (RTC == NRF_RTC2)
	{
		rtc2_delay_completed_flag = false;
		delayFlag = &rtc2_delay_completed_flag;
	}

	RTCEnableComparingReg(RTC, DELAY_REGISTER_MAIN);
	RTC->CC[DELAY_REGISTER_MAIN] = RTC->COUNTER + time_ticks;

	while (!(*delayFlag))
	{
#if SOFTDEVICE_ENABLED
		sd_app_evt_wait();
#else
		__WFE();
#endif
	}

	RTCDisableComparingReg(RTC, DELAY_REGISTER_MAIN);
	return E_RTC_OK;
}

RTC_Error_e RTCTimeout(NRF_RTC_Type* RTC, uint32_t time_ticks, uint8_t* outTimeoutId)
{
	volatile bool* timoutFlag = NULL;


	for (uint8_t i=0; i<RTC_TIMEOUT_ARRAY_SIZE; ++i)
	{
		if (rtcTimeoutArray[i].activeFlag == false)
		{
			RTCEnableComparingReg(RTC, i+1);
			RTC->CC[i+1] = RTC->COUNTER + time_ticks;
			rtcTimeoutArray[i].activeFlag = true;
			rtcTimeoutArray[i].timeoutTriggeredFlag = false;
			*outTimeoutId = i;
		}
	}


	return E_RTC_TIMEOUT;
}

RTC_Error_e RTCClearTimeout(NRF_RTC_Type* RTC, uint8_t timeoutId)
{
	if (RTC == NRF_RTC1)
	{
		rtcTimeoutArray[timeoutId].activeFlag = false;
		RTCDisableComparingReg(RTC, (timeoutId + 1));
	}

	return E_RTC_OK;
}

RTC_Error_e RTCInit(NRF_RTC_Type* RTC)
{
	RTC_Error_e retval = 0;

#if !SOFTDEVICE_ENABLED
	if (RTC == NRF_RTC0)
		return E_RTC_PERIPH_TAKEN_BY_SOFTDEVICE;
#endif

#if SOFTDEVICE_ENABLED
	if (RTC == NRF_RTC1)
	{
		sd_nvic_SetPriority(RTC1_IRQn, RTC1_PRIORITY);
		sd_nvic_EnableIRQ(RTC1_IRQn);
		NRF_RTC1->EVTENSET = RTC_EVTENSET_OVRFLW_Enabled << RTC_EVTENSET_OVRFLW_Pos;
		NRF_RTC1->INTENSET = RTC_INTENSET_OVRFLW_Enabled << RTC_INTENSET_OVRFLW_Pos;
	}

	if (RTC == NRF_RTC2)
	{
		sd_nvic_SetPriority(RTC2_IRQn, RTC2_PRIORITY);
		sd_nvic_EnableIRQ(RTC2_IRQn);
	}
#else
	if (RTC == NRF_RTC0)
	{
		NVIC_SetPriority(RTC1_IRQn, RTC0_PRIORITY);
		NVIC_EnableIRQ(RTC1_IRQn);
	}

	if (RTC == NRF_RTC1)
	{
		NVIC_SetPriority(RTC1_IRQn, RTC1_PRIORITY);
		NVIC_EnableIRQ(RTC1_IRQn);
	}

	if (RTC == NRF_RTC2)
	{
		NVIC_SetPriority(RTC2_IRQn, RTC2_PRIORITY);
		NVIC_EnableIRQ(RTC2_IRQn);
	}
#endif

	RTCStop(RTC);

	RTC->EVTENCLR |= RTC_EVTENCLR_TICK_Enabled << RTC_EVTENCLR_TICK_Pos;
	RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE0_Enabled << RTC_EVTENCLR_COMPARE0_Pos;
	RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE1_Enabled << RTC_EVTENCLR_COMPARE1_Pos;
	RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE2_Enabled << RTC_EVTENCLR_COMPARE2_Pos;

	if (RTC != NRF_RTC0)
		RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE3_Enabled << RTC_EVTENCLR_COMPARE3_Pos;

	switch ((uint32_t)RTC)
	{

		case (uint32_t)NRF_RTC0:
		{
			RTC->PRESCALER = RTC0_PRESCALER;
		}break;

		case (uint32_t)NRF_RTC1:
		{
			RTC->PRESCALER = RTC1_PRESCALER;
		}break;

		case (uint32_t)NRF_RTC2:
		{
			RTC->PRESCALER = RTC2_PRESCALER;
		}break;

		default:
			return E_RTC_PERIPH_TAKEN_BY_SOFTDEVICE;
	}

	RTCStart(RTC);

	return E_RTC_OK;
}

RTC_Error_e RTCEnableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber)
{
	switch (registerNumber)
	{
		case 0:
		{
			RTC->EVTENSET |= RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;
		}break;

		case 1:
		{
			RTC->EVTENSET |= RTC_EVTENSET_COMPARE1_Enabled << RTC_EVTENSET_COMPARE1_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE1_Enabled << RTC_INTENSET_COMPARE1_Pos;
		}break;

		case 2:
		{
			RTC->EVTENSET |= RTC_EVTENSET_COMPARE2_Enabled << RTC_EVTENSET_COMPARE2_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE2_Enabled << RTC_INTENSET_COMPARE2_Pos;
		}break;

		case 3:
		{
			if (RTC == NRF_RTC0)
				return E_RTC_REG_NOT_EXISTING;

			RTC->EVTENSET |= RTC_EVTENSET_COMPARE3_Enabled << RTC_EVTENSET_COMPARE3_Pos;
			RTC->INTENSET |= RTC_INTENSET_COMPARE3_Enabled << RTC_INTENSET_COMPARE3_Pos;
		}break;
	}

	return E_RTC_OK;
}

RTC_Error_e RTCDisableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber)
{
	switch (registerNumber)
	{
		case 0:
		{
			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE0_Enabled << RTC_EVTENCLR_COMPARE0_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE0_Enabled << RTC_INTENCLR_COMPARE0_Pos;
		}break;

		case 1:
		{
			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE1_Enabled << RTC_EVTENCLR_COMPARE1_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE1_Enabled << RTC_INTENCLR_COMPARE1_Pos;
		}break;

		case 2:
		{
			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE2_Enabled << RTC_EVTENCLR_COMPARE2_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE2_Enabled << RTC_INTENCLR_COMPARE2_Pos;
		}break;

		case 3:
		{
			if (RTC == NRF_RTC0)
				return E_RTC_REG_NOT_EXISTING;

			RTC->EVTENCLR |= RTC_EVTENCLR_COMPARE3_Enabled << RTC_EVTENCLR_COMPARE3_Pos;
			RTC->INTENCLR |= RTC_INTENCLR_COMPARE3_Enabled << RTC_INTENCLR_COMPARE3_Pos;
		}break;
	}

	return E_RTC_OK;
}

RTC_Error_e RTCStart(NRF_RTC_Type* RTC)
{
	RTC->TASKS_START = 1;

	return E_RTC_OK;
}

RTC_Error_e RTCStop(NRF_RTC_Type* RTC)
{
	RTC->TASKS_STOP = 1;

	return E_RTC_OK;
}

RTC_Error_e RtcSetTimestamp(uint32_t time)
{
    rtc_timestamp_partial = time - NRF_RTC1->COUNTER/RTC1_FREQUENCY;
    return E_RTC_OK;
}

uint32_t RtcGetTimestamp()
{
    return (rtc_timestamp_partial + NRF_RTC1->COUNTER/RTC1_FREQUENCY);
}

uint32_t RtcConvertDateTimeToTimestamp(rtc_time_t* inTime, rtc_date_t* inDate)
{
    struct tm _time;

    _time.tm_sec = inTime->second;
    _time.tm_min = inTime->minute;
    _time.tm_hour = inTime->hour;
    _time.tm_year = inDate->year - 1900;
    _time.tm_mon = inDate->month - 1;
    _time.tm_mday = inDate->day;
    _time.tm_isdst = -1;

    return mktime(&_time);

}
