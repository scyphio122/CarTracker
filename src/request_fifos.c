/*
 * request_fifos.c
 *
 *  Created on: Oct 31, 2017
 *      Author: Konrad Traczyk
 */

#include "request_fifos.h"
#include "nrf_error.h"
#include <stdint-gcc.h>
#include "ble_uart_service.h"
#include "fifo.h"
#include <string.h>
#include "ble_common.h"
#include "internal_memory_organization.h"
#include "internal_flash.h"
#include "tasks.h"
#include "crypto.h"
#include "gsm.h"
#include "RTC.h"
#include "lsm6dsm.h"
#include "pinout.h"
#include "nrf_gpio.h"

static volatile fifo_t     ble_uart_pending_requests_fifo;
static uint8_t    ble_uart_pending_requests_fifo_buffer[16];
static fifo_t     ble_uart_pending_requests_parameter_fifo;
static uint32_t   ble_uart_pending_requests_parameter_buffer[16];

static fifo_t     system_pending_requests_fifo;
static uint8_t    system_pending_requests_fifo_buffer[16];
static fifo_t     system_pending_requests_parameter_fifo;
static uint32_t   system_pending_requests_parameter_buffer[16];

uint32_t BleUartTaskFifoInit()
{
    FifoInit(&ble_uart_pending_requests_fifo,
            ble_uart_pending_requests_fifo_buffer,
            sizeof(ble_uart_pending_requests_fifo_buffer),
            sizeof(uint8_t));

    FifoInit(&ble_uart_pending_requests_parameter_fifo,
            ble_uart_pending_requests_parameter_buffer,
            sizeof(ble_uart_pending_requests_parameter_buffer),
            sizeof(uint32_t));
}

uint32_t BleUartAddPendingTask(ble_uart_communication_commands_e command)
{
    FifoPut(&ble_uart_pending_requests_fifo, command);

    return NRF_SUCCESS;
}

uint32_t BleUartAddPendingTaskParameters(void* parameterAddress)
{
    FifoPut(&ble_uart_pending_requests_parameter_fifo, (uint32_t)parameterAddress);

    return NRF_SUCCESS;
}

static void* BleUartGetPendingTaskParameter()
{
    uint32_t* address = NULL;
    FifoGet(&ble_uart_pending_requests_parameter_fifo, &address);
    return address;
}

uint32_t BleUartServicePendingTasks()
{
    ble_uart_communication_commands_e command;
    uint32_t* paramAddress = NULL;

    while (!FifoIsEmpty(&ble_uart_pending_requests_fifo))
    {
        FifoGet(&ble_uart_pending_requests_fifo, (uint8_t*)&command);

        switch (command)
        {
            case E_TEST:
            {
                uint32_t size = sizeof("Litwo, Ojczyzno moja! Ile Cie trzeba cenic");
                char* test = malloc(size);
                memcpy(test, "Litwo, Ojczyzno moja! Ile Cie trzeba cenic", size);
                BleUartDataIndicate(m_conn_handle_peripheral, E_TEST, test, size, true);
            }break;

            case E_TEST_ACC_SAMPLES:
            {
                ImuFifoFlush();
                nrf_gpio_pin_clear(DEBUG_ORANGE_LED_PIN);
                nrf_gpio_pin_set(DEBUG_RED_LED_PIN);
                RTC_Error_e err = RTCDelay(NRF_RTC1, RTC1_MS_TO_TICKS(10000));
                uint32_t samplesCount = ImuFifoGetAllSamples(NULL, 0);

                uint32_t timestamp = RtcGetTimestamp();
                uint32_t retval = BleUartDataIndicate(m_conn_handle_peripheral, 1, _imuAccelerometerAxisX, samplesCount*sizeof(uint16_t), false);
                retval = BleUartDataIndicate(m_conn_handle_peripheral, 2, _imuAccelerometerAxisY, samplesCount*sizeof(uint16_t), false);
                retval = BleUartDataIndicate(m_conn_handle_peripheral, 3, _imuAccelerometerAxisZ, samplesCount*sizeof(uint16_t), false);
                BleUartDataIndicate(m_conn_handle_peripheral, 4, &timestamp, sizeof(timestamp), false);

                nrf_gpio_pin_set(DEBUG_ORANGE_LED_PIN);

            }break;

            case E_BLE_UART_SET_DEVICE_PHONE_NUMBER:
            {
                FifoGet(&ble_uart_pending_requests_parameter_fifo, &paramAddress);
                uint64_t devicePhone = 0;
                memcpy(&devicePhone, paramAddress, sizeof(devicePhone));

                IntFlashUpdatePage((uint8_t*)&devicePhone, sizeof(devicePhone), (uint32_t*)GSM_DEVICE_PHONE_NUMBER_ADDRESS);
            }break;

            case E_BLE_UART_SET_OWNER_PHONE_NUMBER:
            {
                FifoGet(&ble_uart_pending_requests_parameter_fifo, &paramAddress);
                uint64_t ownerPhoneNumber = 0;
                memcpy(&ownerPhoneNumber, paramAddress, sizeof(ownerPhoneNumber));

                IntFlashUpdatePage((uint8_t*)&ownerPhoneNumber, sizeof(ownerPhoneNumber), (uint32_t*)GSM_OWNER_PHONE_NUMBER_ADDRESS);
            }break;

            case E_BLE_UART_SET_DEVICE_ID:
            {
                FifoGet(&ble_uart_pending_requests_parameter_fifo, &paramAddress);
                uint32_t deviceId = 0;
                memcpy(&deviceId, paramAddress, sizeof(uint32_t));

                IntFlashUpdatePage((uint8_t*)&deviceId, sizeof(deviceId), (uint32_t*)DEVICE_ID);
            }break;

            case E_BLE_UART_SEND_IV_ON_KEY_TAG_CONNECT:
            {
                uint8_t keySize = 0;
                // Generate new key - initialization vector for connecting with Key Tag
                CryptoGenerateKey(currentInitialisingVector, &keySize);
                uint8_t* encryptedIv = malloc(16);

                CryptoECBEncryptData(currentInitialisingVector, CRYPTO_KEY_SIZE, mainEncryptionKey, CRYPTO_KEY_SIZE, encryptedIv);

                BleUartDataIndicate(m_conn_handle_central, E_BLE_UART_SEND_IV_ON_KEY_TAG_CONNECT | 0x80, encryptedIv, CRYPTO_KEY_SIZE, true);
            }break;

            case E_BLE_UART_GET_AVAILABLE_TRACKS:
            {

            }break;

            case E_BLE_UART_GET_HISTORY_TRACK:
            {

            }break;
        }
    }

    free(paramAddress);
    return NRF_SUCCESS;
}

uint32_t SystemAddPendingTask(system_commands_e command)
{
    FifoPut(&system_pending_requests_fifo, command);

    return NRF_SUCCESS;
}

uint32_t SystemAddPendingTaskParameters(void* parameterAddress)
{
    FifoPut(&system_pending_requests_parameter_fifo, (uint32_t)parameterAddress);

    return NRF_SUCCESS;
}

static void* SystemGetPendingTaskParameter()
{
    uint32_t* address = NULL;
    FifoGet(&system_pending_requests_parameter_fifo, &address);
    return address;
}

uint32_t SystemServicePendingTasks()
{
    system_commands_e command;
    uint32_t* paramAddress = NULL;

    while (!FifoIsEmpty(&system_pending_requests_fifo))
    {
        FifoGet(&system_pending_requests_fifo, (uint8_t*)&command);

        switch (command)
        {
            case E_SYSTEM_SEND_SMS_WITH_LOCATION:
            {
                TaskAlarmSendLocation();
            }break;

            case E_SYSTEM_START_NEW_TRACK:
            {
                TaskStartNewTrack();
            }break;

            case E_SYSTEM_STOP_NEW_TRACK:
            {
                TaskEndCurrentTrack();
            }break;

            case E_SYSTEM_SYNCHRONIZE_TIME_WITH_GSM:
            {
                GsmSynchronizeTime();
            }break;
        }
    }

    free(paramAddress);
    return NRF_SUCCESS;
}


