/*
 * GPIOTE.c
 *
 *  Created on: Oct 30, 2017
 *      Author: Konrad Traczyk
 */
#include "GPIOTE.h"
#include "settings.h"
#include "nrf52.h"
#include "nrf52_bitfields.h"
#include <stdint-gcc.h>
#include "pinout.h"
#include "gsm.h"
#include "nrf_nvic.h"
#include "nrf.h"
#include "nfc.h"
#include "lsm6dsm.h"
#include "tasks.h"
#include "nrf_gpio.h"

void GPIOTE_IRQHandler()
{
    // GSM RING
    if (NRF_GPIOTE->EVENTS_IN[0])
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        GsmSmsReadAll();
    }

    // NFC_INTERRUPT
    if (NRF_GPIOTE->EVENTS_IN[1])
    {
        NRF_GPIOTE->EVENTS_IN[1] = 0;
        NfcIrqCallback();
    }

    // ACCELEROMETER INTERRUPT 1
    if (NRF_GPIOTE->EVENTS_IN[2])
    {
        NRF_GPIOTE->EVENTS_IN[2] = 0;
    }

    // ACCELEROMETER INTERRUPT 2
    if (NRF_GPIOTE->EVENTS_IN[3])
    {
        NRF_GPIOTE->EVENTS_IN[3] = 0;

        if (ImuIsWakeUpIRQ())
        {
            nrf_gpio_pin_clear(DEBUG_ORANGE_LED_PIN);
            TaskStartNewTrack();
        }
    }

    if (NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;
    }
}

void GpioteInit()
{
    sd_nvic_SetPriority(GPIOTE_IRQn, APPLICATION_IRQ_LOWEST_PRIORITY);
    sd_nvic_EnableIRQ(GPIOTE_IRQn);

    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                            (GSM_RING_INT_PIN << GPIOTE_CONFIG_PSEL_Pos);

    NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) |
                            (NFC_IRQ_PIN << GPIOTE_CONFIG_PSEL_Pos);

    NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                            (ACC_INTERRUPT_1_PIN << GPIOTE_CONFIG_PSEL_Pos);

    NRF_GPIOTE->CONFIG[3] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                            (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                            (ACC_INTERRUPT_2_PIN << GPIOTE_CONFIG_PSEL_Pos);

    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos |
                           GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos |
                           GPIOTE_INTENSET_IN2_Enabled << GPIOTE_INTENSET_IN2_Pos |
                           GPIOTE_INTENSET_IN3_Enabled << GPIOTE_INTENSET_IN3_Pos;



}

