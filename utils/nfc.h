/*
 * nfc.h
 *
 *  Created on: Oct 28, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_NFC_H_
#define UTILS_NFC_H_

#include <stdint-gcc.h>
#include <stdbool.h>

/*               ############ NFC REGISTERS DEFINITIONS ############        */

#define NFC_CHIP_STATUS_REG_ADDRESS             (uint8_t)(0x00)
#define NFC_CHIP_STATUS_STANDBY                 (uint8_t)0x80
#define NFC_CHIP_STATUS_LOW_POWER_NO_RX         (uint8_t)0x00
#define NFC_CHIP_STATUS_ONLY_RX                 (uint8_t)0x02
#define NFC_CHIP_STATUS_HALF_POWER              (uint8_t)0x30
#define NFC_CHIP_STATUS_FULL_POWER              (uint8_t)0x20

#define NFC_ISO_CTRL_REGISTER_ADDRESS           (uint8_t)0x01
#define NFC_ISO_ENABLE_RX_CRC                   (uint8_t)0x80
#define NFC_ISO_14443_A_106_KBPS_PROTOCOL       (uint8_t)0x08
#define NFC_ISO_CTRL_DIRECT_MODE_1              (uint8_t)0x40


#define NFC_REGULATOR_AND_IO_REG_ADDRESS        (uint8_t)(0x0B)

#define NFC_IRQ_STATUS_REGISTER_ADDRESS         (uint8_t)(0x0C)
#define NFC_IRQ_NO_RESPONSE_TIME                (uint8_t)0x01
#define NFC_IRQ_COLLISION_ERROR                 (uint8_t)0x02
#define NFC_IRQ_BYTE_FRAMING_EOF_ERROR          (uint8_t)0x04
#define NFC_IRQ_PARITY_ERROR                    (uint8_t)0x08
#define NFC_IRQ_CRC_ERROR                       (uint8_t)0x10
#define NFC_IRQ_FIFO_HIGH_OR_LOW                (uint8_t)0x20
#define NFC_IRQ_RX_START                        (uint8_t)0x40
#define NFC_IRQ_END_OF_TX                       (uint8_t)0x80

// This is TX FIFO
#define NFC_FIFO_ADDRESS                        (uint8_t)(0x1F)

#define NFC_FIFO_STATUS_REG_ADDRESS             (uint8_t)(0x1C)
#define NFC_FIFO_OVERFLOW_FLAG                  (uint8_t)0x10
#define NFC_FIFO_LOW_LEVEL_FLAG                 (uint8_t)0x20
#define NFC_FIFO_LEVEL_HIGH_FLAG                (uint8_t)0x40

/*               ############ NFC COMMANDS DEFINITIONS ############        */

#define NFC_IDLE_COMMAND                        (uint8_t)0x00
#define NFC_SOFTWARE_INIT_COMMAND               (uint8_t)0x03
#define NFC_RESET_FIFO_COMMAND                  (uint8_t)0x0F

#define NFC_TX_WITHOUT_CRC_COMMAND              (uint8_t)0x10
#define NFC_TX_WITH_CRC_COMMAND                 (uint8_t)0x11
#define NFC_DELAYED_TX_WITHOUT_CRC_COMMAND      (uint8_t)0x12
#define NFC_DELAYED_TX_WITH_CRC_COMMAND         (uint8_t)0x13
#define NFC_END_OF_FRAME_COMMAND                (uint8_t)0x14

#define NFC_BLOCK_RX_COMMAND                    (uint8_t)0x16
#define NFC_ENABLE_RX_COMMAND                   (uint8_t)0x17

#define NFC_TEST_INTERNAL_RF_COMMAND            (uint8_t)0x18
#define NFC_TEST_EXTERANL_RF_COMMAND            (uint8_t)0x19
#define NFC_RECEIVER_GAIN_ADJUST_COMMAND        (uint8_t)0x1A


typedef enum
{
    NFC_POWER_OFF,
    NFC_SLEEP,
    NFC_POWER_ON,
    NFC_STANDBY,
    NFC_LOW_POWER,
    NFC_RX_ONLY,
    NFC_HALF_POWER,
    NFC_FULL_POWER
}nfc_state_e;

void NfcWriteRegister(uint8_t registerAddress, uint8_t data);

void NfcReadRegister(uint8_t registerAddress, uint8_t* dataBuf);

void NfcReadBlock(uint8_t startAddress, uint8_t* dataBuf, uint8_t dataSize);

void NfcSendCommand(uint8_t command);

void NfcInit();

void NfcDeinit();

void NfcPowerOn();

void NfcPowerOff();

void NfcSleep();

void NfcStandby();

void NfcLowPower();

void NfcRxOnly();

void NfcTxRxHalfPower();

void NfcTxRxFullPower();

void NfcSetProtocol();

void NfcResetFifo();

uint8_t NfcGetCurrentFifoCounter();

void NfcEnableRxCrcCheck(bool enabled);

void NfcReadIrqStatus(uint8_t* status);

void NfcIrqCallback();

void NfcTransferData(uint8_t* data, uint8_t dataSize);

#endif /* UTILS_NFC_H_ */

