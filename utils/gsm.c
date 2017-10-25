/*
 * gsm.c
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#include "gsm.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "at_commands.h"
#include "nrf_gpio.h"
#include "pinout.h"
#include "UART.h"
#include <stdint-gcc.h>
#include "Systick.h"
#include "fifo.h"
#include "internal_flash.h"
#include "internal_memory_organization.h"
#include <string.h>

void GsmGpsInit()
{
    // VBAT off
    nrf_gpio_cfg_input(GSM_ENABLE_PIN, NRF_GPIO_PIN_NOPULL);

    // Low level for PWR_KEY turns off
    nrf_gpio_cfg_output(GSM_PWRKEY_PIN);
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);

    nrf_gpio_cfg_input(GSM_RING_INT_PIN, NRF_GPIO_PIN_PULLUP);

    GsmBatteryOn();

    UartConfig(UART_BAUDRATE_BAUDRATE_Baud19200,
               UART_CONFIG_PARITY_Excluded,
               UART_CONFIG_HWFC_Disabled);

    GsmPowerOn();
    UartEnable();
    UartRxStart();

    // If fixed baudrate was not yet stored in the GSM ROM
    if (*(uint32_t*)(GSM_BAUDRATE_CONFIG_ADDRESS) == 0xFFFFFFFF)
    {
        uint8_t char1 = 0;
        uint8_t char2 = 0;
        gsm_error_e err;

        do
        {
            // Turn off command echo
            UartSendDataSync("ATE0\n", sizeof("ATE0\n")-1);
            SystickDelayMs(100);
            char1 = FifoPeek(&uartRxFifo, 3);
            char2 = FifoPeek(&uartRxFifo, 2);
        }while(char1 != 'O' && char2 != 'K');

        // Set error rerturn val to verbose string
        GsmUartSendCommand(AT_GSM_ERROR_LOG_LEVEL(GSM_ERROR_LOG_LEVEL_STRING), sizeof(AT_GSM_ERROR_LOG_LEVEL(GSM_ERROR_LOG_LEVEL_STRING)));

        // Enable HW flow control
        err = GsmUartSendCommand(AT_GSM_ENABLE_HW_FLOW_CTRL, sizeof(AT_GSM_ENABLE_HW_FLOW_CTRL));

        if (err == GSM_OK)
        {
            UartEnableFlowCtrl();
        }

        err = GsmUartSendCommand(AT_GSM_CHANGE_BAUD(115200), sizeof(AT_GSM_CHANGE_BAUD(115200)));
        if (err == GSM_OK)
        {
            UartChangeBaudrate(UARTE_BAUDRATE_BAUDRATE_Baud115200);
        }

        // Store the config in the flash not to repeat this procedure every time the
        err = GsmUartSendCommand("AT&W", sizeof("AT&W"));
        if (err == GSM_OK)
        {
            uint32_t t = GSM_FIXED_BAUDRATE_SET;
            IntFlashUpdatePage((uint8_t*)(&t), sizeof(t), GSM_BAUDRATE_CONFIG_ADDRESS);
        }

        FifoClear(&uartRxFifo);
    }
    else
    {
        UartChangeBaudrate(UARTE_BAUDRATE_BAUDRATE_Baud115200);
        UartEnableFlowCtrl();
    }

    GsmUartSendCommand("ATI", sizeof("ATI"));


}

void GsmBatteryOn()
{
    // VBAT on
    nrf_gpio_cfg_output(GSM_ENABLE_PIN);
    nrf_gpio_pin_clear(GSM_ENABLE_PIN);
    SystickDelayMs(100);
}

void GsmBatteryOff()
{
    nrf_gpio_cfg_input(GSM_ENABLE_PIN, NRF_GPIO_PIN_NOPULL);
}

void GsmPowerOn()
{
    // PWRKEY sequence
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);
    SystickDelayMs(2000);
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);
}

void GsmPowerOff()
{
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);
    SystickDelayMs(1000);
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);
    SystickDelayMs(1000);
}

gsm_error_e GsmUartSendCommand(void* command, uint16_t commandSize)
{
    static uint8_t cmd[32];
    uint8_t char1 = 0;
    uint8_t char2 = 0;
    uint8_t char3 = 0;

    memcpy(cmd, command, commandSize - 1);
    cmd[commandSize - 1] = '\n';

    UartEnable();
    UartRxStart();

    UartSendDataSync(cmd, commandSize);

    do
    {
        SystickDelayMs(300);
        char1 = FifoPeek(&uartRxFifo, 4);
        char2 = FifoPeek(&uartRxFifo, 3);
        char3 = FifoPeek(&uartRxFifo, 2);
    }while(!((char2 == 'O' && char3 == 'K') ||
             ((char1 == 'E') && (char2 == 'R') && (char3 == 'R'))));

    UartRxStop();
    UartDisable();

    if (char2 == 'O' && char3 == 'K')
        return GSM_OK;

    return GSM_ERROR;
}

