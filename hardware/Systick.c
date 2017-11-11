#include "Systick.h"
#include <stdbool.h>
#include "system_nrf52.h"
#include "nrf52.h"
#include "settings.h"
#include "nrf_sdm.h"
#include "nrf_nvic.h"
#include "scheduler.h"

#define SYSTICK_CLOCK_FREQ			(64000000ul)
#define SYSTICK_MS_TO_TICKS(x)		((x)*(SYSTICK_CLOCK_FREQ/1000))

static uint32_t systickCnt;
static bool		systickTimeoutFlag;


void SysTick_Handler()
{
	systickCnt--;
	if(systickCnt == 0)
	{
		systickTimeoutFlag = true;
	}

	SchedulerCheckOperations();
}

void SystickInit()
{
	SysTick_Config(SystemCoreClock/1000);

#if !SOFTDEVICE_ENABLED
	NVIC_SetPriority(SysTick_IRQn, 2);
	NVIC_EnableIRQ(SysTick_IRQn);
#else
	bool sdEnabled = false;
	sd_softdevice_is_enabled((uint8_t*)(&sdEnabled));
	if(sdEnabled)
	{
		uint32_t retval = sd_nvic_SetPriority(SysTick_IRQn, APPLICATION_IRQ_LOWEST_PRIORITY);
		retval = sd_nvic_EnableIRQ(SysTick_IRQn);
	}

#endif
}

void SystickDelayMs(uint32_t timeMs)
{
	systickCnt = timeMs;//SYSTICK_MS_TO_TICKS(timeMs);
	systickTimeoutFlag = false;

	while(!systickTimeoutFlag)
	{
		__WFE();
	}
}


