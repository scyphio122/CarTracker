/*
 * ble_central.c
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */
#include "ble_central.h"
#include "ble_uart_service.h"
#include "ble_common.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_db_discovery.h"
#include "app_error.h"
#include "app_timer.h"
#include <stdint-gcc.h>
#include <stdbool.h>
#include <string.h>
#include "ble_uart_service.h"
#include "ble_gattc.h"
#include "nrf_sdh_ble.h"
#include "ble_uart_service_central.h"
#include "crypto.h"

BLE_DB_DISCOVERY_DEF(m_db_disc);                                /**< DB discovery module instance. */


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t _BleCentralParseAdvData(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool _FindAdvName(ble_gap_evt_adv_report_t const * p_adv_report, char const * name_to_find)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    //search for advertising names
    err_code = _BleCentralParseAdvData(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);
    if (err_code == NRF_SUCCESS)
    {
        if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len) == 0)
        {
            return true;
        }
    }
    else
    {
        // Look for the short local name if it was not found as complete
        err_code = _BleCentralParseAdvData(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            return false;
        }
        if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len) == 0)
        {
            return true;
        }
    }
    return false;
}

/**@brief Function for searching a UUID in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * UUID in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   uuid_to_find   UUIID to search.
 * @return   true if the given UUID was found, false otherwise.
 */
static bool _FindAdvUuid(ble_gap_evt_adv_report_t const * p_adv_report, uint16_t uuid_to_find)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     type_data;

    // Initialize advertisement report for parsing.
    adv_data.p_data   = (uint8_t *)p_adv_report->data;
    adv_data.data_len = p_adv_report->dlen;

    err_code = _BleCentralParseAdvData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
                                &adv_data,
                                &type_data);

    if (err_code != NRF_SUCCESS)
    {
        // Look for the services in 'complete' if it was not found in 'more available'.
        err_code = _BleCentralParseAdvData(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
                                    &adv_data,
                                    &type_data);

        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit.
            return false;
        }
    }

    // Verify if any UUID match the given UUID.
    for (uint32_t i = 0; i < (type_data.data_len / sizeof(uint16_t)); i++)
    {
        uint16_t extracted_uuid = uint16_decode(&type_data.p_data[i * sizeof(uint16_t)]);
        if (extracted_uuid == uuid_to_find)
        {
            return true;
        }
    }
    return false;
}

/**@brief Function for checking if a link already exists with a new connected peer.
 *
 * @details This function checks if a newly connected device is already connected
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
bool IsAlreadyConnected(ble_gap_addr_t const * p_connected_adr)
{
    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        if (m_connected_peers[i].is_connected)
        {
            if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type)
            {
                if (memcmp(m_connected_peers[i].address.addr,
                           p_connected_adr->addr,
                           sizeof(m_connected_peers[i].address.addr)) == 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

static void _ServiceDiscoveryHandler(ble_db_discovery_evt_t* p_evt)
{
    ble_uart_on_db_disc_evt(&m_uart_c, p_evt);
}

static void _ServiceDiscoveryInit()
{
    uint32_t err_code = ble_db_discovery_init(_ServiceDiscoveryHandler);
    APP_ERROR_CHECK(err_code);
}

void BleCentralInit()
{
    ble_uart_c_init_t ble_uart_service_central;
    ble_uart_service_central.evt_handler = BleUartCentralHandler;

    _ServiceDiscoveryInit();

    uint32_t retval = BleUartServiceCentralInit(&m_uart_c, &ble_uart_service_central);

}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              should be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t            err_code;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected (HR or RSC), initiate DB
        //  discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            BleCentralScanStop();
            m_conn_handle_central = BLE_CONN_HANDLE_ALL;

            memset(&m_db_disc, 0, sizeof(m_db_disc));
            err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
//            ble_db

        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            m_conn_handle_central = BLE_CONN_HANDLE_INVALID;
            BleCentralScanStart();
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
        {
            if (IsAlreadyConnected(&p_gap_evt->params.adv_report.peer_addr))
            {
                break;
            }

            bool do_connect = false;
            if (strlen(m_target_periph_name) != 0)
            {
                if (_FindAdvName(&p_gap_evt->params.adv_report, m_target_periph_name))
                {
                    do_connect = true;
                }
            }

            if (do_connect)
            {
                err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
                                              &m_scan_params, &m_connection_param,
                                              APP_BLE_CONN_CFG_TAG);

                if (err_code != NRF_SUCCESS)
                {
                    return;
                }

                BleUartAddPendingTask(E_BLE_UART_SEND_IV_ON_KEY_TAG_CONNECT);
            }
        } break; // BLE_GAP_ADV_REPORT

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                BleCentralScanStart();
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function to start scanning.
 */
void BleCentralScanStart(void)
{
    ret_code_t err_code;

    (void) sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
}

void BleCentralScanStop(void)
{
    (void) sd_ble_gap_scan_stop();
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
//static void pm_evt_handler(pm_evt_t const * p_evt)
//{
//    ret_code_t err_code;
//    uint16_t role = ble_conn_state_role(p_evt->conn_handle);
//
//    switch (p_evt->evt_id)
//    {
//        case PM_EVT_BONDED_PEER_CONNECTED:
//        {
//            NRF_LOG_DEBUG("%s : PM_EVT_BONDED_PEER_CONNECTED: peer_id=%d",
//                           nrf_log_push(roles_str[role]),
//                           p_evt->peer_id);
//        } break;
//
//        case PM_EVT_CONN_SEC_START:
//        {
//            NRF_LOG_DEBUG("%s : PM_EVT_CONN_SEC_START: peer_id=%d",
//                           nrf_log_push(roles_str[role]),
//                           p_evt->peer_id);
//        } break;
//
//        case PM_EVT_CONN_SEC_SUCCEEDED:
//        {
//            NRF_LOG_INFO("%s : PM_EVT_CONN_SEC_SUCCEEDED conn_handle: %d, Procedure: %d",
//                           nrf_log_push(roles_str[role]),
//                           p_evt->conn_handle,
//                           p_evt->params.conn_sec_succeeded.procedure);
//        } break;
//
//        case PM_EVT_CONN_SEC_FAILED:
//        {
//            NRF_LOG_DEBUG("%s: PM_EVT_CONN_SEC_FAILED: peer_id=%d, error=%d",
//                          nrf_log_push(roles_str[role]),
//                          p_evt->peer_id,
//                          p_evt->params.conn_sec_failed.error);
//
//        } break;
//
//        case PM_EVT_CONN_SEC_CONFIG_REQ:
//        {
//            // Reject pairing request from an already bonded peer.
//            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
//            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
//        } break;
//
//        case PM_EVT_STORAGE_FULL:
//        {
//            // Run garbage collection on the flash.
//            err_code = fds_gc();
//            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
//            {
//                // Retry.
//            }
//        } break;
//
//        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
//        {
//            NRF_LOG_DEBUG("%s: PM_EVT_PEER_DATA_UPDATE_SUCCEEDED: peer_id=%d data_id=0x%x action=0x%x",
//                           nrf_log_push(roles_str[role]),
//                           p_evt->peer_id,
//                           p_evt->params.peer_data_update_succeeded.data_id,
//                           p_evt->params.peer_data_update_succeeded.action);
//        } break;
//
//        case PM_EVT_PEERS_DELETE_SUCCEEDED:
//        {
//            adv_scan_start();
//        } break;
//
//        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
//        {
//            // The local database has likely changed, send service changed indications.
//            pm_local_database_has_changed();
//        } break;
//
//        case PM_EVT_PEER_DATA_UPDATE_FAILED:
//        {
//            // Assert.
//            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
//        } break;
//
//        case PM_EVT_PEER_DELETE_FAILED:
//        {
//            // Assert.
//            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
//        } break;
//
//        case PM_EVT_PEERS_DELETE_FAILED:
//        {
//            // Assert.
//            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
//        } break;
//
//        case PM_EVT_ERROR_UNEXPECTED:
//        {
//            // Assert.
//            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
//        } break;
//
//        case PM_EVT_PEER_DELETE_SUCCEEDED:
//        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
//        case PM_EVT_SERVICE_CHANGED_IND_SENT:
//        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
//        default:
//            break;
//    }
//}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;
    data_t     adv_data;
    data_t     dev_name;
    bool       do_connect = false;

    // For readibility.
    ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
    ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

    // Initialize advertisement report for parsing
    adv_data.p_data   = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.data_len = p_gap_evt->params.adv_report.dlen;

    // Search for advertising names.
    bool name_found = false;
    err_code = _BleCentralParseAdvData(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);

    if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete.
        err_code = _BleCentralParseAdvData(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit
            return;
        }
        else
        {
            name_found = true;
        }
    }
    else
    {
        name_found = true;
    }

    if (name_found)
    {
        if (strlen(m_target_periph_name) != 0)
        {
            if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.data_len )== 0)
            {
                do_connect = true;
            }
        }
    }

    if (do_connect)
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(peer_addr,
                                      &m_scan_params,
                                      &m_connection_param,
                                      APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
}

