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

static void _GsmWaitForNetworkLogging()
{
    char* index = NULL;
    do
    {
        sd_app_evt_wait();

        index = strstr((const char*)uartRxFifo.p_buf, "RDY");
    }while(index == NULL);
    FifoClear(&uartRxFifo);
}

static void _GsmWaitForInitStart()
{
    char  response[32];
    memset(response, 0, sizeof(response));
    char* index = NULL;
    gsm_error_e err = GSM_OK;
    do
    {
        err = GsmUartSendCommand(AT_GSM_QUERY_INITIALIZATION_STATUS, sizeof(AT_GSM_QUERY_INITIALIZATION_STATUS), response);
        index = strstr(uartRxFifo.p_buf, ":");
        index += 2;
        if (index != NULL && (*index != '3'))
        {
            SystickDelayMs(100);
            continue;
        }
        else
        {
            break;
        }

    } while (1);
}

void GsmGpsInit()
{
    // VBAT off
    nrf_gpio_cfg_input(GSM_ENABLE_PIN, NRF_GPIO_PIN_NOPULL);

    // Low level for PWR_KEY turns off
    nrf_gpio_cfg_output(GSM_PWRKEY_PIN);
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);

    nrf_gpio_cfg_input(GSM_RING_INT_PIN, NRF_GPIO_PIN_PULLUP);

    GsmBatteryOn();

    UartConfig(UART_BAUDRATE_BAUDRATE_Baud115200,
               UART_CONFIG_PARITY_Excluded,
               UART_CONFIG_HWFC_Disabled);

    UartEnable();
    UartRxStart();

    GsmPowerOn(true);


    // If fixed baudrate was not yet stored in the GSM ROM
    if (*(uint32_t*)(GSM_BAUDRATE_CONFIG_ADDRESS) == 0xFFFFFFFF)
    {
        uint8_t char1 = 0;
        uint8_t char2 = 0;
        gsm_error_e err;

        // Block command echo
        while (GsmUartSendCommand("ATE0", sizeof("ATE0"), NULL) != GSM_OK)
        {
            SystickDelayMs(10);
        }

        // Set error rerturn val to verbose string
        GsmUartSendCommand(AT_GSM_ERROR_LOG_LEVEL(GSM_ERROR_LOG_LEVEL_STRING), sizeof(AT_GSM_ERROR_LOG_LEVEL(GSM_ERROR_LOG_LEVEL_STRING)), NULL);

        // Enable HW flow control
        err = GsmUartSendCommand(AT_GSM_ENABLE_HW_FLOW_CTRL, sizeof(AT_GSM_ENABLE_HW_FLOW_CTRL), NULL);

        if (err == GSM_OK)
        {
            UartEnableFlowCtrl();
        }

        err = GsmUartSendCommand(AT_GSM_CHANGE_BAUD(115200), sizeof(AT_GSM_CHANGE_BAUD(115200)), NULL);
        if (err == GSM_OK)
        {
            UartChangeBaudrate(UARTE_BAUDRATE_BAUDRATE_Baud115200);
        }

        // Store the config in the flash not to repeat this procedure every time the
        err = GsmUartSendCommand("AT&W", sizeof("AT&W"), NULL);
        if (err == GSM_OK)
        {
            uint32_t t = GSM_FIXED_BAUDRATE_SET;
            IntFlashUpdatePage((uint8_t*)(&t), sizeof(t), GSM_BAUDRATE_CONFIG_ADDRESS);
        }
    }
    else
    {
        UartChangeBaudrate(UARTE_BAUDRATE_BAUDRATE_Baud115200);
        UartEnableFlowCtrl();
    }

    _GsmWaitForInitStart();

    GsmSmsInit();

    GsmBlockIncommingCalls();


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

void GsmPowerOn(bool waitForLogon)
{
    // PWRKEY sequence
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);
    SystickDelayMs(2000);
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);

    if (waitForLogon)
        _GsmWaitForNetworkLogging();
}

void GsmPowerOff()
{
    nrf_gpio_pin_set(GSM_PWRKEY_PIN);
    SystickDelayMs(1000);
    nrf_gpio_pin_clear(GSM_PWRKEY_PIN);
    SystickDelayMs(1000);
}

gsm_error_e GsmUartSendCommand(void* command, uint16_t commandSize, char* response)
{
    static uint8_t cmd[32];
    char* success = NULL;
    char* error = NULL;
    memcpy(cmd, command, commandSize - 1);
    cmd[commandSize - 1] = '\n';
    FifoClear(&uartRxFifo);
    memset(uartRxFifo.p_buf, 0, uartRxFifo.buf_size_mask);
    UartEnable();
    UartRxStart();

    UartSendDataSync(cmd, commandSize);

    do
    {
        sd_app_evt_wait();
        success = strstr(uartRxFifo.p_buf, "OK");
        error = strstr(uartRxFifo.p_buf, "ERROR");
    }while(success == NULL && error == NULL);

    UartRxStop();
    UartDisable();


    if (response != NULL && success)
    {
        // Point to the '\r\n' before the OK response
        success -= 2;
        char* temp = uartRxFifo.p_buf;
        while (temp != success)
        {
            *response = *temp;
            response++;
            temp++;
        }
    }

    if (success)
        return GSM_OK;

    return GSM_ERROR;
}

void GsmBlockIncommingCalls()
{
    GsmUartSendCommand(AT_GSM_SET_REFUSE_OPTS(GSM_RECEIVE_SMS, GSM_REFUSE_INCOMMING_CALL),
                       sizeof(AT_GSM_SET_REFUSE_OPTS(GSM_RECEIVE_SMS, GSM_REFUSE_INCOMMING_CALL)), NULL);
}

void GsmSmsInit()
{
    GsmUartSendCommand(AT_GSM_SMS_SET_FORMAT_TEXT, sizeof(AT_GSM_SMS_SET_FORMAT_TEXT), NULL);
    GsmUartSendCommand(AT_GSM_SET_SMS_CHARSET("GSM"), sizeof(AT_GSM_SET_SMS_CHARSET("GSM")), NULL);
}

void GsmSmsSend(char* telNum, const char* text)
{
    char textMsg[256];

    sprintf(textMsg, "%s\"%s\"\r%s%c%c", AT_GSM_SEND_SMS_MESSAGE, telNum, text, '\x1A', '\0');

    GsmUartSendCommand(textMsg, strlen(textMsg) + 1, NULL);
}

