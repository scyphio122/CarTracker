/*
 * ble_uart_service_central.c
 *
 *  Created on: Sep 30, 2017
 *      Author: Konrad Traczyk
 *      Email : k.traczyk@mudita.com
 */
#include "ble_uart_service_central.h"
#include "ble_uart_service.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
//#include "ble_srv_common.h"
#include "ble_gattc.h"
#include <string.h>
#include "crypto.h"
#include "RTC.h"
#include "ble_hci.h"
#include "tasks.h"
#include "ble_common.h"
#include "pinout.h"
#include "nrf_gpio.h"

#define TX_BUFFER_MASK         7                     /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
    READ_REQ,  /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ  /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of this message, i.e. read or write message. */
    union
    {
        uint16_t       read_handle;  /**< Read request message. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;

static tx_message_t   m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t       m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t       m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */

static uint8_t        m_uart_central_tx_buffer[20];
/**@brief Running Speed and Cadence Collector Handler.
 */
void BleUartCentralHandler(ble_uart_c_t * p_uart_c, ble_uart_c_evt_t * p_uart_c_evt)
{
    ret_code_t err_code;

    switch (p_uart_c_evt->evt_type)
    {
        case BLE_UART_EVT_DISCOVERY_COMPLETE:
            // Initiate bonding.
            err_code = ble_uart_c_handles_assign(&m_uart_c,
                                                  p_uart_c_evt->conn_handle,
                                                 &p_uart_c_evt->rx_params.uart_db,
                                                 &p_uart_c_evt->tx_params.uart_db,
                                                 &p_uart_c_evt->notify_params.uart_db);
            APP_ERROR_CHECK(err_code);

            // Running Speed and Cadence service discovered. Enable Running Speed and Cadence notifications.
            err_code =  ble_uart_c_indicate_enable(p_uart_c);
            //APP_ERROR_CHECK(err_code);
            nrf_gpio_pin_clear(DEBUG_ORANGE_LED_PIN);
            BleUartAddPendingTask(E_BLE_UART_SEND_IV_ON_KEY_TAG_CONNECT);
            break;

        case BLE_UART_EVT_INDICATION_RECEIVED:
        {
//            p_uart_c_evt->params.uart_packet.command
            uint8_t request_code = p_uart_c_evt->tx_params.uart_packet.command;
            uint8_t packet_size = p_uart_c_evt->tx_params.uart_packet.packet_size;
            uint8_t packet[16];

            sd_ble_gattc_hv_confirm(p_uart_c_evt->conn_handle, p_uart_c_evt->tx_params.uart_db.uart_handle);

            // If data is encrypted - decrypt it
//            if (request_code & 0x80)
//            {
                CryptoCFBDecryptData(   p_uart_c_evt->tx_params.uart_packet.data,
                                        CryptoGetCurrentInitialisingVector(),
                                        mainEncryptionKey,
                                        CRYPTO_KEY_SIZE,
                                        packet,
                                        16);
//            }a
//            else
//            {
//                memcpy(packet, p_uart_c_evt->tx_params.uart_packet.data, sizeof(packet));
//            }

            switch(request_code)
            {
                case E_BLE_UART_DEACTIVATE_ALARM:
                {
                    if (memcmp(alarmDeactivationCmd, packet, CRYPTO_KEY_SIZE) == 0)
                    {
                        TaskDeactivateAlarm();
                    }

                    sd_ble_gap_disconnect(m_conn_handle_central, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION );

                }break;

                default:
                {

                }break;
            }
        }break;

        default:
            break;
    }
}

/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
//            NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
//                          "attempted again..");
        }
    }
}


/**@brief     Function for handling write response events.
 *
 * @param[in] p_ble_rscs_c Pointer to the Running Speed and Cadence Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_write_rsp(ble_uart_c_t * p_ble_uart_c, const ble_evt_t * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_uart_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if there is any message to be sent across to the peer and send it.
    tx_buffer_process();
}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the Running Speed and Cadence measurement from
 *            the peer. If it is, this function will decode the Running Speed measurement and send it
 *            to the application.
 *
 * @param[in] p_ble_rscs_c Pointer to the Running Speed and Cadence Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_hvx(ble_uart_c_t * p_ble_uart_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_hvx_t * p_notif = &p_ble_evt->evt.gattc_evt.params.hvx;

    // Check if the event if on the link for this instance
    if (p_ble_uart_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    // Check if this is a Running Speed and Cadence notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_uart_c->tx_peer_db.uart_handle)
    {
        uint32_t         index = 0;
        ble_uart_c_evt_t ble_uart_c_evt;
        ble_uart_c_evt.evt_type    = BLE_UART_EVT_INDICATION_RECEIVED;
        ble_uart_c_evt.conn_handle = p_ble_evt->evt.gattc_evt.conn_handle;
        ble_uart_c_evt.tx_params.uart_packet.command        = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
        ble_uart_c_evt.tx_params.uart_packet.packet_size    = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
        memcpy( &ble_uart_c_evt.tx_params.uart_packet.data,
                &p_ble_evt->evt.gattc_evt.params.hvx.data[2],
                p_ble_evt->evt.gattc_evt.params.hvx.data[1]);

        //lint -save -e415 -e416 -e662 "Access of out of bounds pointer" "Creation of out of bounds pointer"

        p_ble_uart_c->evt_handler(p_ble_uart_c, &ble_uart_c_evt);

        //lint -restore
    }
}


/**@brief     Function for handling events from the database discovery module.
 *
 * @details   This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of heart rate service at the peer. If so, it will
 *            call the application's event handler indicating that the Running Speed and Cadence
 *            service has been discovered at the peer. It also populates the event with the service
 *            related information before providing it to the application.
 *
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 *
 */
void ble_uart_on_db_disc_evt(ble_uart_c_t * p_ble_uart_c, const ble_db_discovery_evt_t * p_evt)
{
    // Check if the Heart Rate Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UART_SERVICE_UUID &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
    {
        // Find the CCCD Handle of the Heart Rate Measurement characteristic.
        uint32_t i;

        ble_uart_c_evt_t evt;

        evt.evt_type    = BLE_DB_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                    TX_CHAR_UUID)
            {
                // Found Uart TX (indicate) characteristic. Store CCCD handle and break.
                evt.tx_params.uart_db.uart_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.tx_params.uart_db.uart_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                    RX_CHAR_UUID)
            {
                // Found Uart RX (write) characteristic. Store CCCD handle and break.
                evt.rx_params.uart_db.uart_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.rx_params.uart_db.uart_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }

            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                    DEV_EVENTS_UUID)
            {
                // Found Uart Events (notify) characteristic. Store CCCD handle and break.
                evt.notify_params.uart_db.uart_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.notify_params.uart_db.uart_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
            }
        }

        //If the instance has been assigned prior to db_discovery, assign the db_handles
//        if (p_ble_uart_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_uart_c->rx_peer_db.uart_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_uart_c->rx_peer_db.uart_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_uart_c->rx_peer_db = evt.rx_params.uart_db;
            }
            if ((p_ble_uart_c->tx_peer_db.uart_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_uart_c->tx_peer_db.uart_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_uart_c->tx_peer_db = evt.tx_params.uart_db;
            }

            if ((p_ble_uart_c->notify_peer_db.uart_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_uart_c->notify_peer_db.uart_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_uart_c->notify_peer_db = evt.notify_params.uart_db;
            }
        }
        evt.evt_type = BLE_UART_EVT_DISCOVERY_COMPLETE;
        p_ble_uart_c->evt_handler(p_ble_uart_c, &evt);
    }
}


uint32_t BleUartServiceCentralInit(ble_uart_c_t * p_ble_uart_c, ble_uart_c_init_t * p_ble_uart_c_init)
{
//    VERIFY_PARAM_NOT_NULL(p_ble_uart_c);
//    VERIFY_PARAM_NOT_NULL(p_ble_uart_c_init);

    ble_uuid_t uart_uuid;

    uart_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    uart_uuid.uuid = BLE_UART_SERVICE_UUID;

    p_ble_uart_c->evt_handler                       = p_ble_uart_c_init->evt_handler;
    p_ble_uart_c->conn_handle                       = BLE_CONN_HANDLE_INVALID;
    p_ble_uart_c->rx_peer_db.uart_cccd_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_uart_c->rx_peer_db.uart_handle            = BLE_GATT_HANDLE_INVALID;
    p_ble_uart_c->tx_peer_db.uart_cccd_handle       = BLE_GATT_HANDLE_INVALID;
    p_ble_uart_c->tx_peer_db.uart_handle            = BLE_GATT_HANDLE_INVALID;
    p_ble_uart_c->notify_peer_db.uart_cccd_handle   = BLE_GATT_HANDLE_INVALID;
    p_ble_uart_c->notify_peer_db.uart_handle        = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&uart_uuid);
}


uint32_t ble_uart_c_handles_assign(ble_uart_c_t *    p_ble_uart_c,
                                   uint16_t          conn_handle,
                                   ble_uart_c_db_t * p_rx_peer_handles,
                                   ble_uart_c_db_t * p_tx_peer_handles,
                                   ble_uart_c_db_t * p_notify_peer_handles)
{
//    VERIFY_PARAM_NOT_NULL(p_ble_uart_c);
    p_ble_uart_c->conn_handle = conn_handle;
    if (p_rx_peer_handles != NULL)
    {
        p_ble_uart_c->rx_peer_db = *p_rx_peer_handles;
    }

    if (p_tx_peer_handles != NULL)
    {
        p_ble_uart_c->tx_peer_db = *p_tx_peer_handles;
    }

    if (p_notify_peer_handles != NULL)
    {
        p_ble_uart_c->notify_peer_db = *p_notify_peer_handles;
    }

    return NRF_SUCCESS;
}


/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function check if the disconnect event is happening on the link
 *            associated with the current instance of the module, if so it will set its
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_rscs_c Pointer to the RSC Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_uart_c_t * p_ble_uart_c, const ble_evt_t * p_ble_evt)
{
    if (p_ble_uart_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_uart_c->conn_handle                       = BLE_CONN_HANDLE_INVALID;
        p_ble_uart_c->rx_peer_db.uart_cccd_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_uart_c->rx_peer_db.uart_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_uart_c->tx_peer_db.uart_cccd_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_uart_c->tx_peer_db.uart_handle            = BLE_GATT_HANDLE_INVALID;
        p_ble_uart_c->notify_peer_db.uart_cccd_handle   = BLE_GATT_HANDLE_INVALID;
        p_ble_uart_c->notify_peer_db.uart_handle        = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_uart_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_uart_c_t * p_ble_uart_c = (ble_uart_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_uart_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_uart_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_uart_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, bool enable)
{
    tx_message_t * p_msg;
    uint16_t       cccd_val = enable ? BLE_GATT_HVX_INDICATION : 0;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}


uint32_t ble_uart_c_indicate_enable(ble_uart_c_t * p_ble_uart_c)
{
//    VERIFY_PARAM_NOT_NULL(p_ble_uart_c);

    if (p_ble_uart_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return cccd_configure(p_ble_uart_c->conn_handle, p_ble_uart_c->tx_peer_db.uart_cccd_handle, true);
}

uint32_t BleUartCentralSendCommand(uint8_t command, uint8_t* data_to_send, uint8_t data_size)
{
    if (m_uart_c.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (data_size > 19)
    {
        return NRF_ERROR_NO_MEM;
    }

    ble_gattc_write_params_t write_data;

    memset(&write_data, 0, sizeof(write_data));

    m_uart_central_tx_buffer[0] = command;
    m_uart_central_tx_buffer[1] = data_size;

    memcpy(&m_uart_central_tx_buffer[2], data_to_send, data_size);

    write_data.handle = m_uart_c.rx_peer_db.uart_handle;
    write_data.write_op = BLE_GATT_OP_WRITE_REQ;
    write_data.flags = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_CANCEL;
    write_data.len = data_size + 2;
    write_data.p_value = m_uart_central_tx_buffer;
    write_data.offset = 0;

    uint32_t retval = sd_ble_gattc_write(m_uart_c.conn_handle, &write_data);

    if (retval != 0)
    {
        nrf_gpio_pin_clear(DEBUG_RED_LED_PIN);
    }
    return  retval;
}
/** @}
 *  @endcond
 */



