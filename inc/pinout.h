/*
 * pinout.h
 *
 *  Created on: Jun 15, 2017
 *      Author: root
 */

#ifndef INC_PINOUT_H_
#define INC_PINOUT_H_

/** ######## DEBUG PINS ######## **/
#define TRACE_SWO_PIN_PIN       (uint8_t)(18)
#define DEBUG_RED_LED_PIN         (uint8_t)(22)
#define DEBUG_ORANGE_LED_PIN    (uint8_t)(23)

/**< ########### UART ############# **/
#define UART_RX_PIN 	        (uint8_t)29
#define UART_TX_PIN		        (uint8_t)28
#define UART_RTS_PIN	        (uint8_t)25
#define UART_CTS_PIN	        (uint8_t)26

/**< ########## SPI ############# **/
#define SPI0_SCK_PIN	        (uint32_t)0xFFFFFFFF
#define SPI0_MISO_PIN	        (uint32_t)0xFFFFFFFF
#define SPI0_MOSI_PIN	        (uint32_t)0xFFFFFFFF

#define SPI1_SCK_PIN	        (uint8_t)19
#define SPI1_MOSI_PIN           (uint8_t)20
#define SPI1_MISO_PIN	        (uint8_t)21

#define SPI2_SCK_PIN	        (uint8_t)8
#define SPI2_MISO_PIN	        (uint8_t)6
#define SPI2_MOSI_PIN	        (uint8_t)7

/**< ######## NFC PINS ########## **/
#define NFC_EN_PIN              (uint8_t)(2)
#define NFC_EN2_PIN             (uint8_t)(3)
#define NFC_MOD_PIN             (uint8_t)(13)
#define NFC_IRQ_PIN             (uint8_t)(14)
#define NFC_ASK_OOK_PIN         (uint8_t)(15)
#define NFC_CS_PIN              (uint8_t)(4)
#define NFC_MISO_PIN            SPI1_MISO_PIN
#define NFC_MOSI_PIN            SPI1_MOSI_PIN
#define NFC_SCK_PIN             SPI1_SCK_PIN
#define NFC_SPI_PERIPH          NRF_SPI1
/**< ######## ACCELERATOR PINS ######## **/
#define ACC_INTERRUPT_1_PIN     (uint8_t)(5)
#define ACC_INTERRUPT_2_PIN     (uint8_t)(10)
#define ACC_ENABLE_PIN          (uint8_t)(12)
#define ACC_CS_PIN              (uint8_t)(9)
#define ACC_MISO_PIN            SPI2_MISO_PIN
#define ACC_MOSI_PIN            SPI2_MOSI_PIN
#define ACC_SCK_PIN             SPI2_SCK_PIN
#define ACC_SPI_PERIPH          NRF_SPI2

/** ######## GSM PINS ######## **/
#define GSM_PWRKEY_PIN          (uint8_t)(11)
#define GSM_RING_INT_PIN        (uint8_t)(27)
#define GSM_ENABLE_PIN          (uint8_t)(30)
#define GSM_TXD_PIN             UART_TX_PIN
#define GSM_RXD_PIN             UART_RX_PIN
#define GSM_RTS_PIN             UART_RTS_PIN
#define GSM_CTS_PIN             UART_CTS_PIN
#define GSM_UART_PERIPH         NRF_UARTE0

/** ######## GPS PINS ######## **/
#define GPS_ENABLE_PIN          (uint8_t)(31)

/** ######## EXTERNAL FLASH ######## **/
#define EXT_FLASH_ENABLE_PIN    (uint8_t)(16)
#define EXT_FLASH_CS_PIN        (uint8_t)(17)
#define EXT_FLASH_SCK_PIN       SPI1_SCK_PIN
#define EXT_FLASH_MOSI_PIN      SPI1_MOSI_PIN
#define EXT_FLASH_MISO_PIN      SPI1_MISO_PIN
#define EXT_FLASH_SPI_PERIPH    NRF_SPI1





#endif /* INC_PINOUT_H_ */
