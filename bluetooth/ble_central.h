/*
 * ble_central.h
 *
 *  Created on: Sep 19, 2017
 *      Author: root
 */

#ifndef BLUETOOTH_BLE_CENTRAL_H_
#define BLUETOOTH_BLE_CENTRAL_H_

#include "ble_common.h"
#include "ble.h"
#include <stdbool.h>

#define KEY_TAG_NAME                                "KeyTag"

#define BLE_CENTRAL_SCAN_INTERVAL                   0x00A0                              /**< Determines scan interval in units of 0.625 millisecond. */
#define BLE_CENTRAL_SCAN_WINDOW                     0x0050                              /**< Determines scan window in units of 0.625 millisecond. */
#define BLE_CENTRAL_SCAN_TIMEOUT                    0x0000                              /**< Timout when scanning. 0x0000 disables timeout. */


#define BLE_CENTRAL_MIN_CONNECTION_INTERVAL         MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines minimum connection interval in milliseconds. */
#define BLE_CENTRAL_MAX_CONNECTION_INTERVAL         MSEC_TO_UNITS(30, UNIT_1_25_MS)     /**< Determines maximum connection interval in milliseconds. */
#define BLE_CENTRAL_SLAVE_LATENCY                   0                                   /**< Determines slave latency in terms of connection events. */
#define BLE_CENTRAL_SUPERVISION_TIMEOUT             MSEC_TO_UNITS(4000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE                                 2                                   /**< Size of a UUID, in bytes. */

typedef struct
{
    bool           is_connected;
    ble_gap_addr_t address;
} conn_peer_t;

/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = 1,
    .interval = BLE_CENTRAL_SCAN_INTERVAL,
    .window   = BLE_CENTRAL_SCAN_WINDOW,
    .timeout  = BLE_CENTRAL_SCAN_TIMEOUT,
    .use_whitelist = 0,
};

/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    BLE_CENTRAL_MIN_CONNECTION_INTERVAL,
    BLE_CENTRAL_MAX_CONNECTION_INTERVAL,
    BLE_CENTRAL_SLAVE_LATENCY,
    BLE_CENTRAL_SUPERVISION_TIMEOUT
};

static char * roles_str[] =
{
    "INVALID_ROLE",
    "CENTRAL",
    "PERIPHERAL",
};

/**@brief names which the central applications will scan for, and which will be advertised by the peripherals.
 *  if these are set to empty strings, the UUIDs defined below will be used
 */
static const char m_target_periph_name[] = KEY_TAG_NAME;


conn_peer_t        m_connected_peers[NRF_BLE_LINK_COUNT];

bool IsAlreadyConnected(ble_gap_addr_t const * p_connected_adr);
void BleCentralInit();
void BleCentralScanStart(void);
void BleCentralScanStop(void);

void on_ble_central_evt(ble_evt_t const * p_ble_evt);
void on_adv_report(const ble_evt_t * const p_ble_evt);

#endif /* BLUETOOTH_BLE_CENTRAL_H_ */
