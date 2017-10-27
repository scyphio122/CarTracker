/*
 * gps.c
 *
 *  Created on: Oct 23, 2017
 *      Author: Konrad Traczyk
 */

#include "gps.h"
#include "at_commands.h"
#include "gsm.h"
#include "UART.h"
#include "pinout.h"
#include "nrf_gpio.h"

void GpsPowerOn()
{
    gsm_error_e err = GSM_OK;
    nrf_gpio_cfg_input(GPS_ENABLE_PIN, NRF_GPIO_PIN_PULLUP);
    uint8_t state = nrf_gpio_pin_read(GPS_ENABLE_PIN);
    GsmUartSendCommand(AT_GPS_POWER_READ, sizeof(AT_GPS_POWER_READ));
    GsmUartSendCommand(AT_GPS_POWER_ON, sizeof(AT_GPS_POWER_ON));

    while (nrf_gpio_pin_read(GPS_ENABLE_PIN) == 0)
    {

    }

    do
    {
        err = GsmUartSendCommand("AT", sizeof("AT"));
    } while (err != GSM_OK);
}

void GpsPowerOff()
{
    GsmUartSendCommand(AT_GPS_POWER_OFF, sizeof(AT_GPS_POWER_OFF));
}

void GpsDisableEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_DISABLE, sizeof(AT_GPS_EPO_DISABLE));
}

void GpsEnableEPO()
{
    GsmUartSendCommand(AT_GPS_EPO_ENABLE, sizeof(AT_GPS_EPO_ENABLE));
}

void GpsGetData()
{
    GsmUartSendCommand(AT_GPS_GET_NAVI_DATA, sizeof(AT_GPS_GET_NAVI_DATA));
}
