/*
 * at_commands.h
 *
 *  Created on: Oct 14, 2017
 *      Author: Konrad Traczyk
 */

#ifndef UTILS_AT_COMMANDS_H_
#define UTILS_AT_COMMANDS_H_
/** -------------------------------------------------------- GSM COMMANDS ------------------------------------------------------------------ */

/** ####### GENERAL COMMANDS ######### **/
#define AT_GSM_ENABLE_HW_FLOW_CTRL          "AT+IFC=2,2"
#define AT_GSM_GET_BAUD                     "AT+IPR?"
#define AT_GSM_CHANGE_BAUD(baud)            "AT+IPR=" #baud
#define AT_GSM_GET_IMEI                     "AT+GSN"

#define GSM_POWER_DOWN_MODE_URGENT          "0"
#define GSM_POWER_DOWN_MODE_NORMAL          "1"
#define AT_GSM_POWER_DOWN(mode)             "AT+QPOWD=" mode

#define GSM_ERROR_LOG_LEVEL_DISABLE         "0"
#define GSM_ERROR_LOG_LEVEL_NUMERIC         "1"
#define GSM_ERROR_LOG_LEVEL_STRING          "2"
#define AT_GSM_ERROR_LOG_LEVEL(lvl)         "AT+CMEE=" lvl

/** ####### GSM COMMANDS ######## **/
#define AT_GSM_ENABLE_SIM_DETECTION         "AT+QSIMDET"
#define AT_GSM_GET_PIN_STATUS               "AT+CPIN?"
#define AT_GSM_SEND_PIN(pin)                "AT+CPIN=" #pin

#define AT_GSM_GET_REGISTERED_GSM_OPERATOR  "AT+COPS?"

#define GSM_RECEIVE_SMS                     "0"
#define GSM_REFUSE_SMS                      "1"
#define GSM_RECEIVE_INCOMMING_CALL          "0"
#define GSM_REFUSE_INCOMMING_CALL           "1"
#define AT_GSM_SET_REFUSE_OPTS(sms,call)    "AT+QREFUSECS=" sms ","  call

#define AT_GSM_QUERY_INITIALIZATION_STATUS  "AT+QINISTAT"

#define AT_GSM_QUERY_NETWORK_STATUS         "AT+QNSTATUS"

#define AT_GSM_QUERY_NETWORK_TIME_INIT      "AT+QNITZ?"
#define AT_GSM_WRITE_NETWORK_TIME_INIT(en)  "AT+QNITZ=" ## (en)

#define AT_GSM_QUERY_LAST_NETWORK_TIME      "AT+QLTS?"


#define SMS_CENTER_NUMBER                   "\"+48790998250\""
#define AT_GSM_SMS_SET_SMS_CENTER_NUMBER    "AT+CSCA=" SMS_CENTER_NUMBER ",145"

#define AT_GSM_SMS_QUERY_FORMAT             "AT+CMGF?"
#define AT_GSM_SMS_SET_FORMAT_PDU           "AT+CMGF=0"
#define AT_GSM_SMS_SET_FORMAT_TEXT          "AT+CMGF=1"

#define AT_GSM_SET_SMS_CHARSET(charset)     "AT+CSCS=" #charset

#define AT_GSM_READ_SMS_MESSAGE             "AT+CMGR="
#define AT_GSM_READ_ALL_SMS_MESSAGES        "AT+CMGL=\"REC UNREAD\""
#define AT_GSM_DELETE_SMS_MESSAGE           "AT+CMGD="
#define AT_GSM_DELETE_ALL_SMS_MESSAGES      "AT+QMGDA=\"DEL ALL\""
#define AT_GSM_SEND_SMS_MESSAGE             "AT+CMGS="


/** -------------------------------------------------------- GPS COMMANDS ------------------------------------------------------------------ */

#define AT_GPS_POWER_READ                   "AT+QGNSSC?"
#define AT_GPS_POWER_ON                     "AT+QGNSSC=1"
#define AT_GPS_POWER_OFF                    "AT+QGNSSC=0"


/** This command makes the GPS to send navigational information */
#define AT_GPS_GET_NAVI_DATA                "AT+QGNSSRD?"
#define AT_GPS_GET_NAVI_MSG(msg)            "AT+QGNSSRD=\"NMEA/" msg "\""

/** Send manufactorer command to the GPS **/
#define AT_GPS_SEND_COMMAND_HEX(cmd)        ("AT+QGNSSCMD=1," cmd)
#define AT_GPS_SEND_COMMAND_NMEA(cmd)       ("AT+QGNSSCMD=0," cmd)

/** Enable EPO functionality **/
#define AT_GPS_EPO_ENABLE                   "AT+QGNSSEPO=1"
#define AT_GPS_EPO_DISABLE                  "AT+QGNSSEPO=0"
#define AT_GPS_EPO_EXECUTE                  "AT+QGEPOAID"

#define AT_GPS_GET_TIME_SYNC_STATUS         "AT+QGNSSTS?"

/** Set reference localization (last known localization) to decrease TTFF **/
#define AT_GPS_SET_REF_LOCATION(lat, lng)   "AT+QGREFLOC=" lat "," lng

#endif /* UTILS_AT_COMMANDS_H_ */
