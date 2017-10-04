/*
 ============================================================================
 Name        : main.c
 Author      : Konrad Traczyk
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C
 ============================================================================
 */

#include <stdio.h>
#include <stdint-gcc.h>
#include "nrf_gpio.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "core_cm4.h"
#include "Systick.h"
#include "nrf_sdm.h"
#include "settings.h"
#include "nrf_nvic.h"
#include "UART.h"
#include "SPI.h"
#include "ble_common.h"
#include "advertising.h"
#include <string.h>
#include "RTC.h"
#include "ble_uart_service.h"
#include "internal_flash.h"
#include "nrf_sdh_soc.h"
#include "ble_central.h"
#include "crypto.h"

/*
 *
 * Print a greeting message on standard output and exit.
 *
 * On embedded platforms this might require semi-hosting or similar.
 *
 * For example, for toolchains derived from GNU Tools for Embedded,
 * to enable semi-hosting, the following was added to the linker:
 *
 * --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
 *
 * Adjust it for other toolchains.
 *
 */

nrf_nvic_state_t nrf_nvic_state = {0};

void POWER_CLOCK_IRQHandler()
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	nrf_gpio_cfg_output(20);
	nrf_gpio_pin_clear(20);
}

/**@brief Function for handling SOC events.
 *
 * @param[in]   evt_id      SOC stack event id.
 * @param[in]   p_context   Unused.
 */
static void soc_evt_handler(uint32_t evt_id, void * p_context)
{
    SD_flash_operation_callback(evt_id);
}

NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);

void SDFaultHandler(uint32_t id, uint32_t pc, uint32_t info)
{
	nrf_gpio_cfg_output(19);
	nrf_gpio_pin_clear(19);
	while(1)
	{

	}
}

void NVICInit()
{
	__enable_irq();
	NVIC_SetPriorityGrouping(0);
}

__attribute__((optimize("O0")))
int main(void)
{
#if SOFTDEVICE_ENABLED
	BleStackInit();

    RTCInit(NRF_RTC1);
    SystickInit();

	GapParamsInit();
	GattInit();
//	ConnParamsInit();
	ServicesInit();
	AdvertisingInit();
	BleCentralInit();

//	AdvertisingStart();
//    BleCentralScanStart();
#endif

    // Check if the Main Key exists
    if (!CryptoCheckMainKey())
    {
        CryptoGenerateAndStoreMainKey();
    }

    uint8_t data[16] = "DEADBEEFABBABAAB";
    uint8_t dataEncrypted[16];
    uint8_t dataDecrypted[16];

    memset(dataEncrypted, 0, 16);
    IntFlashErasePage((uint32_t*)0x30000);
    uint32_t encryptStart = NRF_RTC1->COUNTER;
    CryptoEncryptData(data, 16, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, 16, dataEncrypted);
    uint32_t encryptEnd = NRF_RTC1->COUNTER;
    IntFlashUpdatePage(dataEncrypted, 16, (uint32_t*)0x30000);

    memset(dataEncrypted, 0, 16);
    memset(dataDecrypted, 0, 16);

    memcpy(dataEncrypted, (uint8_t*)0x30000, 16);
    uint32_t decryptStart = NRF_RTC1->COUNTER;
    CryptoDecryptData(dataEncrypted, 16, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, 16, dataDecrypted);
    uint32_t decryptEnd = NRF_RTC1->COUNTER;

    uint32_t encryptTime = (encryptEnd - encryptStart);
    uint32_t decryptTime = decryptEnd - decryptStart;

//	uint32_t retcode = 0;
//	retcode = IntFlashStoreWord(0xDEADBEEF, (uint32_t*)0x30000);
//	retcode = IntFlashStoreWord(0x12345678, (uint32_t*)0x30004);
//	retcode = IntFlashErasePage((uint32_t*)0x30000);
//	UartConfig(UART_BAUDRATE_BAUDRATE_Baud9600, UART_CONFIG_PARITY_Included, UART_CONFIG_HWFC_Disabled);
//	UartEnable();
//	UartSendDataSync("Hello World, it's nRF52!", sizeof("Hello World, it's nRF52!"));
//
//	SpiConfig(NRF_SPI0, SPI_FREQUENCY_FREQUENCY_M8, SPI_CONFIG_ORDER_MsbFirst, SPI_CONFIG_CPHA_Leading, SPI_CONFIG_CPOL_ActiveHigh);
//	SpiEnable(NRF_SPI0);
//	SpiWrite(NRF_SPI0, "Hello World, it's nRF52!", sizeof("Hello World, it's nRF52!"));
//	UartReadDataEndCharSync(buf, '\n');M

	nrf_gpio_cfg_output(17);
	while(1)
	{
//	    BleUartServicePendingTasks();
//		RTCDelay(NRF_RTC1, RTC1_MS_TO_TICKS(1000));
//		nrf_gpio_pin_toggle(17);
	}

  return 0;
}
