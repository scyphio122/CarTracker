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

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
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
#include "external_flash_driver.h"
#include "pinout.h"
#include "gsm.h"
#include "gps.h"
#include "GPIOTE.h"
#include "scheduler.h"
#include "fifo.h"
#include "request_fifos.h"
#include "file_system.h"
#include "lsm6dsm.h"
//#include "nfc.h"
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

void InitDeviceData()
{
    memcpy(&gsmDeviceNumber     , (uint32_t*)GSM_DEVICE_PHONE_NUMBER_ADDRESS   , sizeof(uint64_t));
    memcpy(&gsmOwnerDeviceNumber, (uint32_t*)GSM_OWNER_PHONE_NUMBER_ADDRESS    , sizeof(uint64_t));
//    memcpy(&deviceId            , (uint32_t*)DEVICE_ID                         , sizeof(uint32_t));
    deviceId = 2;
    memcpy(mainEncryptionKey    , (uint32_t*)CRYPTO_MAIN_KEY_ADDRESS           , CRYPTO_KEY_SIZE);

//    Mem_Org_Init();
}

extern void initialise_monitor_handles(void);

__attribute__((optimize("O0")))
int main(void)
{
NRF_CLOCK->TRACECONFIG = 0;

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

	InitDeviceData();

//	AdvertisingStart();
//    BleCentralScanStart();
#endif
	nrf_gpio_cfg_output(DEBUG_1_PIN_PIN);
	nrf_gpio_pin_set(DEBUG_1_PIN_PIN);
//
	nrf_gpio_cfg_output(DEBUG_2_PIN_PIN);
	nrf_gpio_pin_clear(DEBUG_2_PIN_PIN);

	uint8_t data = 0x44;

//    GsmGpsInit();


	ImuInit();

	ImuTurnOn();

    GpioteInit();

    ImuFifoConfigure();

	imu_sample_set_t sample;
	uint8_t reg_value = 0;
	uint16_t buf[64];
	int cnt = 0;
	memset(buf, 0, sizeof(buf));

	ImuFifoFlush();
	do
	{

//	    ImuReadRegister(WAKE_UP_SRC_REG, &reg_value, sizeof(reg_value));
	    buf[cnt++] = ImuFifoGetSamplesCount();
	    ImuReadRegister(OUT_X_G_L, &sample, sizeof(sample));
	    SystickDelayMs(100);

	    if (cnt == 64)
	    {
	        nrf_gpio_pin_clear(DEBUG_2_PIN_PIN);
	    }
	}while (1);

//	do
//	{
// 	    ImuReadRegister(WHO_AM_I_REG, (uint8_t*)&data, 1);
//	    SystickDelayMs(10);
//	} while (1);
////
//    while(1)
//        __WFE();

//	rtc_time_t time;
//	rtc_date_t date;
//	time.hour = 21;
//	time.minute = 18;
//	time.second = 34;
//
//	date.day = 30;
//	date.month = 10;
//	date.year = 2017;
//	uint32_t timestamp = RtcConvertDateTimeToTimestamp(&time, &date);
//	IntFlashErasePage((uint32_t*)PERSISTENT_CONFIG_PAGE_ADDRESS);

//	gps_coord_t lat;
//	gps_coord_t lon;
//
//	lat.degrees = 52;
//	lat.minutes = 10;
//	lat.seconds = 8058;
//
//	lon.degrees = 21;
//	lon.minutes = 3;
//	lon.seconds = 4160;
//
//    GpsPowerOn();
//
//	GpsSetReferencePosition(&lat, &lon);
//	GpsAgpsTrigger();
//
//      do
//      {
////          GpsGetData();
//          GpsRequestMessage(GPS_MSG_GGA);
//          if (gpsLastSample.fixStatus != GPS_FIX_NO_FIX && gpsLastSample.fixStatus != 0)
//          {
//              nrf_gpio_pin_clear(DEBUG_1_PIN_PIN);
//              nrf_gpio_pin_set(DEBUG_2_PIN_PIN);
//              break;
//          }
//
//          SystickDelayMs(60000);
//      }while (1);




//
//    if (!CryptoCheckMainKey())
//    {
//        CryptoGenerateAndStoreMainKey();
//    }
////


//	NfcInit();
//
//	NfcPowerOn();
//	SystickDelayMs(1);
//	NfcTxRxHalfPower();
//	ExtFlashInit();
//
//    ExtFlashTurnOn(EXT_FLASH_PROGRAM_OP);
//    uint8_t data[64] = "Litwo, Ojczyzno moja, Ty jestes jak zdrowie, Ten tylko sie dowie";
//    uint8_t b[64];
//    ExtFlashProgramPageThroughBufferWithoutPreerase(0x1000, data, 64);
//    ExtFlashReadPage(0x1000, b, 64);

    // Check if the Main Key exists




//    uint8_t dataEncrypted[64];
//    uint8_t dataDecrypted[64];
//    uint8_t iv[16];
//    uint8_t ivSize;
//
//    CryptoGenerateKey((uint8_t*)iv, &ivSize);
//    memset(dataEncrypted, 0, 64);
////    ExtFlashProgramPageThroughBufferWithoutPreerase(0x30000, data, 64);
//    IntFlashErasePage((uint32_t*)0x30000);
////    uint8_t d[64];
//
////    ExtFlashReadPage(0x3000, d, 64);
//
//    uint32_t encryptStart = NRF_RTC1->COUNTER;
//    CryptoCFBEncryptData(data, iv, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, 16, dataEncrypted, 64);
//    //CryptoEncryptData(data, 16, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, 16, dataEncrypted);
//    uint32_t encryptEnd = NRF_RTC1->COUNTER;
//
//    IntFlashUpdatePage(dataEncrypted, 64, (uint32_t*)0x30000);
//
//    memset(dataEncrypted, 0, 64);
//    memset(dataDecrypted, 0, 64);
//
//    memcpy(dataEncrypted, (uint8_t*)0x30000, 64);
//    uint32_t decryptStart = NRF_RTC1->COUNTER;
//    CryptoCFBDecryptData(dataEncrypted, iv, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, 16, dataDecrypted, 64);
////    CryptoECBDecryptData(dataEncrypted, 16, (uint8_t*)CRYPTO_MAIN_KEY_ADDRESS, 16, dataDecrypted);
//    uint32_t decryptEnd = NRF_RTC1->COUNTER;
//
//    uint32_t encryptTime = (encryptEnd - encryptStart);
//    uint32_t decryptTime = decryptEnd - decryptStart;

	TaskStartCarMovementDetection();

	while(1)
	{
	    sd_app_evt_wait();

	    ScheduleExecutePendingOperations();
	    BleUartServicePendingTasks();
        SystemServicePendingTasks();
	}

  return 0;
}


