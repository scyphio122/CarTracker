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


void GsmInit()
{
    nrf_gpio_cfg_output(GSM_ENABLE_PIN);
    nrf_gpio_pin_set(GSM_ENABLE_PIN);

    nrf_gpio_cfg_output(GSM_PWRKEY_PIN);
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);

    nrf_gpio_cfg_input(GSM_RING_INT_PIN);

    UartConfig(UART_BAUDRATE_BAUDRATE_Baud19200,
               UART_CONFIG_PARITY_Excluded,
               UART_CONFIG_HWFC_Enabled);

    nrf_gpio_pin_clear(GSM_ENABLE_PIN);
    SystickDelayMs(100);

    GsmUartSendCommand(AT_GSM_CHANGE_BAUD("115200"), sizeof(AT_GSM_CHANGE_BAUD("115200")));

    // Enable HW flow control
    GsmUartSendCommand(AT_GSM_ENABLE_HW_FLOW_CTRL, sizeof(AT_GSM_ENABLE_HW_FLOW_CTRL));
}

void GsmPowerOn()
{

    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);
    SystickDelayMs(1000);
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);
}

void GsmPowerOff()
{
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);
    SystickDelayMs(1000);
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);
}

void GsmUartSendCommand(void* command, uint16_t commandSize)
{
    UartSendDataSync(command, commandSize);
}

