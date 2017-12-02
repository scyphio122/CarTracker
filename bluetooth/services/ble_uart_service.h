/*
 * ble_uart_service.h
 *
 *  Created on: Sep 21, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */

#ifndef BLUETOOTH_SERVICES_BLE_UART_SERVICE_H_
#define BLUETOOTH_SERVICES_BLE_UART_SERVICE_H_

#include <stdint-gcc.h>
#include <ble.h>
#include <ble_gatts.h>
#include <ble_srv_common.h>

#define SECUCAR_UUID_BASE           {(uint8_t)0xb0, (uint8_t)0x07, (uint8_t)0x20, (uint8_t)0xac, (uint8_t)0xca, (uint8_t)0x16, (uint8_t)0x20, (uint8_t)0x3c, (uint8_t)0xb9, (uint8_t)0xe7, (uint8_t)0x14, (uint8_t)0x72, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x04, (uint8_t)0xac}

#define BLE_UART_SERVICE_UUID       (0x0001)
#define RX_CHAR_UUID                (0x0002)
#define TX_CHAR_UUID                (0x0003)
#define DEV_EVENTS_UUID             (0x0004)

typedef enum
{
    E_TEST,
    E_TEST_ACC_SAMPLES = 12,
    E_BLE_UART_GET_TIMESTAMP,
    E_BLE_UART_SET_TIMESTAMP,
    E_BLE_UART_GET_DEVICE_ID,
    E_BLE_UART_SET_DEVICE_ID,
    E_BLE_UART_GET_OWNER_PHONE_NUMBER,
    E_BLE_UART_SET_OWNER_PHONE_NUMBER,
    E_BLE_UART_GET_DEVICE_PHONE_NUMBER,
    E_BLE_UART_SET_DEVICE_PHONE_NUMBER,

    E_BLE_UART_SEND_IV_ON_KEY_TAG_CONNECT,
    E_BLE_UART_DEACTIVATE_ALARM,

    E_BLE_UART_GET_AVAILABLE_TRACKS,
    E_BLE_UART_GET_HISTORY_TRACK
}ble_uart_communication_commands_e;

typedef enum
{
    BLE_UART_EVT_INDICATION_ENABLED,                                         /**< BLE UART value indication enabled event. */
    BLE_UART_EVT_INDICATION_DISABLED,                                        /**< BLE UART value indication disabled event. */
    BLE_UART_EVT_NOTIFICATION_ENABLED,
    BLE_UART_EVT_NOTIFICATION_DISABLED,
    BLE_UART_EVT_NOTIFICATION_TRANSMITTED,
    BLE_UART_EVT_INDICATION_CONFIRMED,                                       /**< Confirmation of a BLE UART indication has been received. */
    BLE_UART_EVT_RX_DATA_RECEIVED
} ble_uart_evt_type_t;

/**@brief Structure containing the handles related to the Heart Rate Service found on the peer. */
typedef struct
{
    uint16_t cccd_handle;  /**< Handle of the CCCD of the Heart Rate Measurement characteristic. */
    uint16_t handle;       /**< Handle of the Heart Rate Measurement characteristic as provided by the SoftDevice. */
} ble_uart_db_t;

/**@brief BLE UART Service event. */
typedef struct
{
    ble_uart_evt_type_t  evt_type;         /**< Type of event. */
    uint16_t             conn_handle;      /**< Connection handle on which the Heart Rate service was discovered on the peer device..*/
    union
    {
        ble_uart_db_t  peer_db;            /**< TX related handles found on the peer device.. This will be filled if the evt_type is @ref BLE_HRS_C_EVT_DISCOVERY_COMPLETE.*/
    } tx_char_params;

    union
    {
        ble_uart_db_t  peer_db;            /**< RX related handles found on the peer device.. This will be filled if the evt_type is @ref BLE_HRS_C_EVT_DISCOVERY_COMPLETE.*/
    } rx_char_params;

    union
    {
        ble_uart_db_t  peer_db;            /**< Events related handles found on the peer device.. This will be filled if the evt_type is @ref BLE_HRS_C_EVT_DISCOVERY_COMPLETE.*/
    } events_char_params;

} ble_uart_evt_t;

typedef uint8_t* ble_uart_data_t;

// Forward declaration of the ble_uart_t type.
typedef struct ble_uart_s ble_uart_t;

/**@brief BLE UART Service event handler type. */
typedef void (*ble_uart_evt_handler_t) (ble_uart_t * p_uart, ble_uart_evt_t * p_evt, ble_uart_data_t p_data, uint8_t data_size);

/**@brief BLE UART Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_uart_evt_handler_t       evt_handler;                /**< Event handler to be called for handling events in the Data Transfer Service. */
    ble_srv_cccd_security_mode_t uart_meas_attr_md;          /**< Initial security level for Data Transfer measurement attribute */
    ble_srv_security_mode_t      uart_feature_attr_md;       /**< Initial security level for Data Transfer feature attribute */
    uint16_t                     feature;                    /**< Initial value for Data Transfer feature */
} ble_uart_init_t;

/**@brief Data Transfer Service structure. This contains various status information for
 *        the service. */
struct ble_uart_s
{
    ble_uart_evt_handler_t       evt_handler;               /**< Event handler to be called for handling events in the Data Transfer Service. */
    uint16_t                     service_handle;            /**< Handle of Data Transfer Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     tx_handles;                /**< Handles related to the Data Transfer TX characteristic. */
    ble_gatts_char_handles_t     rx_handles;                /**< Handles related to the Data Transfer RX characteristic. */
    ble_gatts_char_handles_t     dev_events_handles;        /**< Handles of Alarms characteristics */
    uint8_t                      uuid_type;
    uint16_t                     conn_handle;               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     feature;                   /**< Value of Data Transfer feature. */
    uint32_t                     alarm_flags;               /**< Error alarm flags */
};

typedef uint8_t ble_command_t ;

/**@brief Data Transfer data typedef. This is a Data Transfer
 *        data struct. */
typedef struct
{
    ble_command_t command_code;
    uint16_t number;
    uint8_t* data_pointer;
}ble_uart_data_to_send_t;

extern ble_uart_t                   m_ble_uart;

uint32_t BleUartServiceInit(ble_uart_t * p_uart, const ble_uart_init_t * p_uart_init);
uint32_t BleUartDataIndicate( uint16_t conn_handle, uint8_t command_code, void* data, uint16_t data_size, uint8_t data_buf_dynamically_allocated);
uint32_t BleUartWaitForIndicateEnd();
uint32_t BleUartDataSendNotify(uint16_t conn_handle, uint8_t command_code, uint8_t* data, uint16_t actual_data_size, uint8_t data_buf_dynamically_allocated);
void BleUartHandler(ble_uart_t * p_uart, ble_uart_evt_t * p_evt, ble_uart_data_t p_data, uint8_t data_size);
uint32_t BleUartIsIndicateEnabled(ble_uart_t * p_uart, bool * p_indication_enabled);
uint32_t BleUartIsNotifyEnabled(ble_uart_t * p_uart, bool * p_indication_enabled);
uint32_t BleUartAddPendingTask(ble_uart_communication_commands_e command);
uint32_t BleUartServicePendingTasks();

void BleUartOnBleEvt(ble_uart_t * p_uart, ble_evt_t const * p_ble_evt);


#endif /* BLUETOOTH_SERVICES_BLE_UART_SERVICE_H_ */
