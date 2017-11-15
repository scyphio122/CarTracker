/*
 * RTC.h
 *
 *  Created on: Sep 9, 2017
 *      Author: root
 */

#ifndef HARDWARE_RTC_H_
#define HARDWARE_RTC_H_

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "settings.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include <stdint-gcc.h>

#define RTC0_PRESCALER				0
#define RTC0_FREQUENCY   			(32768/(RTC0_PRESCALER + 1))
#define RTC0_MS_TO_TICKS(time_ms)	(time_ms*RTC0_FREQUENCY/1000)
#define RTC0_US_TO_TICKS(time_us)	(time_us*RTC0_FREQUENCY/1000000)
#define RTC0_S_TO_TICKS(time_s)      (time_s*RTC0_FREQUENCY)

#define RTC1_PRESCALER				0
#define RTC1_FREQUENCY   			(32768/(RTC1_PRESCALER + 1))
#define RTC1_MS_TO_TICKS(time_ms)	(time_ms*RTC1_FREQUENCY/1000)
#define RTC1_US_TO_TICKS(time_us)	(time_us*RTC1_FREQUENCY/1000000)
#define RTC1_S_TO_TICKS(time_s)     (time_s*RTC1_FREQUENCY)


#define RTC2_PRESCALER				327 // RTC2 Frequency = 99.9 Hz
#define RTC2_FREQUENCY   			(32768/(RTC2_PRESCALER + 1))
#define RTC2_MS_TO_TICKS(time_ms)	(time_ms*RTC2_FREQUENCY/1000)
#define RTC2_US_TO_TICKS(time_us)	(time_us*RTC2_FREQUENCY/1000000)
#define RTC2_S_TO_TICKS(time_s)     (time_s*RTC2_FREQUENCY)

#define RTC_TIMEOUT_ARRAY_SIZE      3

typedef enum
{
	E_RTC_OK = 0,
	E_RTC_REG_NOT_EXISTING,
	E_RTC_PERIPH_TAKEN_BY_SOFTDEVICE,
	E_RTC_TIMEOUT,
}RTC_Error_e;

typedef struct
{
	bool activeFlag;
	bool timeoutTriggeredFlag;
}rtc_timeout_t;

typedef struct
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
}rtc_time_t;

typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
}rtc_date_t;



extern volatile rtc_timeout_t rtcTimeoutArray[RTC_TIMEOUT_ARRAY_SIZE];

RTC_Error_e RTCInit(NRF_RTC_Type* RTC);
RTC_Error_e RTCEnableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber);
RTC_Error_e RTCDisableComparingReg(NRF_RTC_Type* RTC, uint8_t registerNumber);
RTC_Error_e RTCStart(NRF_RTC_Type* RTC);
RTC_Error_e RTCStop(NRF_RTC_Type* RTC);

RTC_Error_e RTCDelay(NRF_RTC_Type* RTC, uint32_t time_ticks);
RTC_Error_e Rtc1DelayMs(uint32_t ms);
RTC_Error_e Rtc2DelayMs(uint32_t ms);

RTC_Error_e RTCTimeout(NRF_RTC_Type* RTC, uint32_t time_ticks, uint8_t* outTimeoutId);
RTC_Error_e RTCClearTimeout(NRF_RTC_Type* RTC, uint8_t timeoutId);

RTC_Error_e RtcSetTimestamp(uint32_t time);
uint32_t RtcGetTimestamp();
uint32_t RtcConvertDateTimeToTimestamp(rtc_time_t* inTime, rtc_date_t* inDate);


#endif /* HARDWARE_RTC_H_ */
