/*
 * nfc.c
 *
 *  Created on: Oct 28, 2017
 *      Author: Konrad Traczyk
 */

#include "nfc.h"
#include "nrf_gpio.h"
#include "pinout.h"
#include "SPI.h"
#include "RTC.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include "Systick.h"
#include "fifo.h"
#include "app_fifo.h"

static uint8_t _nfcFifoBuffer[32];
app_fifo_t nfcFifo;

static nfc_state_e _nfcState = NFC_POWER_OFF;

void NfcWriteRegister(uint8_t registerAddress, uint8_t data)
{
    // Clear the necessary bits
    registerAddress &= ~(0xE0);

    SpiEnable(NFC_SPI_PERIPH);
    SpiCSAssert(NFC_CS_PIN);

    SpiSwitchPolarityPhase(NFC_SPI_PERIPH, SPI_CONFIG_CPOL_ActiveHigh, SPI_CONFIG_CPHA_Leading);
    SpiWrite(NFC_SPI_PERIPH, &registerAddress, sizeof(registerAddress));
    SpiWrite(NFC_SPI_PERIPH, &data, sizeof(data));

    SpiCSDeassert(NFC_CS_PIN);
    SpiDisable(NFC_SPI_PERIPH);

}

void NfcReadRegister(uint8_t registerAddress, uint8_t* dataBuf)
{
    // Clear the necessary bits
    registerAddress &= ~(0xE0);
    // Set the read access
    registerAddress |= 0x40;

    SpiEnable(NFC_SPI_PERIPH);
    SpiCSAssert(NFC_CS_PIN);

    SpiSwitchPolarityPhase(NFC_SPI_PERIPH, SPI_CONFIG_CPOL_ActiveHigh, SPI_CONFIG_CPHA_Leading);
    SpiWrite(NFC_SPI_PERIPH, &registerAddress, sizeof(registerAddress));

    SpiSwitchPolarityPhase(NFC_SPI_PERIPH, SPI_CONFIG_CPOL_ActiveHigh, SPI_CONFIG_CPHA_Trailing);
    SpiRead(NFC_SPI_PERIPH, dataBuf, sizeof(uint8_t));

    SpiCSDeassert(NFC_CS_PIN);
    SpiDisable(NFC_SPI_PERIPH);
}

void NfcReadBlock(uint8_t startAddress, uint8_t* dataBuf, uint8_t dataSize)
{
    // Clear the necessary bits
    startAddress &= ~(0xE0);
    // Set the read access and continuous mode
    startAddress |= 0x60;

    SpiEnable(NFC_SPI_PERIPH);
    SpiCSAssert(NFC_CS_PIN);

    SpiSwitchPolarityPhase(NFC_SPI_PERIPH, SPI_CONFIG_CPOL_ActiveHigh, SPI_CONFIG_CPHA_Leading);
    SpiWrite(NFC_SPI_PERIPH, &startAddress, sizeof(startAddress));

    SpiSwitchPolarityPhase(NFC_SPI_PERIPH, SPI_CONFIG_CPOL_ActiveHigh, SPI_CONFIG_CPHA_Trailing);
    SpiRead(NFC_SPI_PERIPH, dataBuf, dataSize);

    SpiCSDeassert(NFC_CS_PIN);
    SpiDisable(NFC_SPI_PERIPH);
}

void NfcSendCommand(uint8_t command)
{
    // Clear the necessary bits
    command &= ~(0xE0);
    // Mark the data as command
    command |= 0x80;

    char _cmd[2];
    _cmd[0] = command;
    SpiSwitchPolarityPhase(NFC_SPI_PERIPH, SPI_CONFIG_CPOL_ActiveHigh, SPI_CONFIG_CPHA_Leading);
    SpiEnable(NFC_SPI_PERIPH);
    SpiCSAssert(NFC_CS_PIN);

    SpiWrite(NFC_SPI_PERIPH, _cmd, sizeof(_cmd));

    SpiCSDeassert(NFC_CS_PIN);
    SpiDisable(NFC_SPI_PERIPH);
}

void NfcInit()
{
    nrf_gpio_cfg_output(NFC_EN_PIN);
    nrf_gpio_cfg_output(NFC_EN2_PIN);
    nrf_gpio_cfg_input(NFC_MOD_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(NFC_ASK_OOK_PIN, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_cfg_sense_input(NFC_IRQ_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

    nrf_gpio_cfg_output(NFC_CS_PIN);
    nrf_gpio_pin_set(NFC_CS_PIN);


    SpiConfig(NFC_SPI_PERIPH,
            E_SPI_FREQUENCY_2_MHz,
            SPI_CONFIG_ORDER_MsbFirst,
            SPI_CONFIG_CPOL_ActiveHigh,
            SPI_CONFIG_CPHA_Leading,
            NFC_CS_PIN);

    NfcPowerOff();

    FifoInit(&nfcFifo, _nfcFifoBuffer, sizeof(_nfcFifoBuffer));
}

void NfcDeinit()
{
    nrf_gpio_cfg_sense_input(NFC_IRQ_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_NOSENSE);
    SpiDisable(NFC_SPI_PERIPH);
}

void NfcPowerOn()
{
    nrf_gpio_pin_set(NFC_EN_PIN);

    NfcLowPower();

    NfcSetProtocol();

    NfcEnableRxCrcCheck(true);
}

void NfcPowerOff()
{
    nrf_gpio_pin_clear(NFC_EN_PIN);
    nrf_gpio_pin_clear(NFC_EN2_PIN);

    _nfcState = NFC_POWER_OFF;
}

void NfcSleep()
{
    nrf_gpio_pin_clear(NFC_EN_PIN);
    nrf_gpio_pin_set(NFC_EN2_PIN);

    _nfcState = NFC_SLEEP;
}

void NfcStandby()
{
    SystickDelayMs(5);
    NfcWriteRegister(NFC_CHIP_STATUS_REG_ADDRESS, NFC_CHIP_STATUS_STANDBY);

    _nfcState = NFC_STANDBY;

}

void NfcLowPower()
{
    RTCDelay(NRF_RTC1, RTC1_US_TO_TICKS(32));
    NfcWriteRegister(NFC_CHIP_STATUS_REG_ADDRESS, NFC_CHIP_STATUS_LOW_POWER_NO_RX);

    _nfcState = NFC_LOW_POWER;
}

void NfcRxOnly()
{
    RTCDelay(NRF_RTC1, RTC1_US_TO_TICKS(32));
    NfcWriteRegister(NFC_CHIP_STATUS_REG_ADDRESS, NFC_CHIP_STATUS_ONLY_RX);

    _nfcState = NFC_RX_ONLY;
}

void NfcTxRxHalfPower()
{
    RTCDelay(NRF_RTC1, RTC1_US_TO_TICKS(32));
    NfcWriteRegister(NFC_CHIP_STATUS_REG_ADDRESS, NFC_CHIP_STATUS_HALF_POWER);

    _nfcState = NFC_HALF_POWER;
}

void NfcTxRxFullPower()
{
    RTCDelay(NRF_RTC1, RTC1_US_TO_TICKS(32));
    NfcWriteRegister(NFC_CHIP_STATUS_REG_ADDRESS, NFC_CHIP_STATUS_FULL_POWER);

    _nfcState = NFC_FULL_POWER;
}

void NfcSetProtocol()
{
    uint8_t regValue = 0;
    NfcReadRegister(NFC_ISO_CTRL_REGISTER_ADDRESS, &regValue);

    regValue &= 0b11100000;
    regValue |= NFC_ISO_14443_A_106_KBPS_PROTOCOL;

    NfcWriteRegister(NFC_ISO_CTRL_REGISTER_ADDRESS, regValue);
}

void NfcEnableRxCrcCheck(bool enabled)
{
    uint8_t regValue = 0;
    NfcReadRegister(NFC_ISO_CTRL_REGISTER_ADDRESS, &regValue);

    if (enabled)
    {
        regValue |= NFC_ISO_ENABLE_RX_CRC;
    }
    else
    {
        regValue &= ~NFC_ISO_ENABLE_RX_CRC;
    }

    NfcWriteRegister(NFC_ISO_CTRL_REGISTER_ADDRESS, regValue);
}

void NfcResetFifo()
{
    NfcSendCommand(NFC_RESET_FIFO_COMMAND);
}

uint8_t NfcGetCurrentFifoCounter()
{
    uint8_t buf[2];
    NfcReadRegister(NFC_FIFO_STATUS_REG_ADDRESS, buf);

    return (buf[0] & 0x0F) + 1;
}

void NfcReadIrqStatus(uint8_t* status)
{
    // Needed due to the NFC_IRQ_STATUS register additional dummy read
    char dummy[2];
    NfcReadBlock(NFC_IRQ_STATUS_REGISTER_ADDRESS, dummy, 2);
    *status = dummy[1];
}

void NfcIrqCallback()
{
    uint8_t irqStatus = 0;
    uint8_t mask = 1;
    NfcReadIrqStatus(&irqStatus);

    for (uint8_t i=0; i<8; ++i)
    {
        switch (irqStatus & mask)
        {
            case NFC_IRQ_NO_RESPONSE_TIME:
            {

            }break;

            case NFC_IRQ_COLLISION_ERROR:
            {

            }break;

            case NFC_IRQ_BYTE_FRAMING_EOF_ERROR:
            {

            }break;

            case NFC_IRQ_PARITY_ERROR:
            {

            }break;

            case NFC_IRQ_CRC_ERROR:
            {

            }break;

            case NFC_IRQ_FIFO_HIGH_OR_LOW:
            {

            }break;

            case NFC_IRQ_RX_START:
            {

                NfcResetFifo();
            }break;

            case NFC_IRQ_END_OF_TX:
            {
                NfcResetFifo();
            }break;

        }

        mask = 1 << i;
    }
}

