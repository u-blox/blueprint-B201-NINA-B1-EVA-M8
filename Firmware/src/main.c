/*
 * Copyright (C) u-blox 
 * 
 * u-blox reserves all rights in this deliverable (documentation, software, etc.,
 * hereafter “Deliverable”). 
 * 
 * u-blox grants you the right to use, copy, modify and distribute the
 * Deliverable provided hereunder for any purpose without fee.
 * 
 * THIS DELIVERABLE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF THIS
 * DELIVERABLE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 * 
 * In case you provide us a feedback or make a contribution in the form of a
 * further development of the Deliverable (“Contribution”), u-blox will have the
 * same rights as granted to you, namely to use, copy, modify and distribute the
 * Contribution provided to us for any purpose without fee.
 */

/**@brief B201 Location demo application main file.
 *
 * This file contains the source code for the B201 Location demo application using the Location and Navigation service
 * (and also Device Information services).
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_lns.h"
#include "ble_dis.h"
#include "sensorsim.h"
#include "app_timer.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT      1                                               /**< Include or not the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE                 GATT_MTU_SIZE_DEFAULT                           /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED            BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2            /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT                   0                                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1                                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME_PREFIX                   "B201-"                                         /**< Name prefix, 4 chars from the MAC will be added as well. */
#define MANUFACTURER_NAME                    "u-blox"                                        /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUMBER                         "B201"                                          /**< Model number. Will be passed to Device Information Service. */
#define SW_VERSION                           "0.1.3"                                         /**< Software version. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                     40                                              /**< The advertising interval (in units of 0.625 ms; this value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                             /**< The advertising time-out in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                               /**< Size of timer operation queues. */

#define LOC_AND_NAV_DATA_INTERVAL            APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)      /**< Location and Navigation data interval (ticks). */
#define SPI_READ_INTERVAL                    APP_TIMER_TICKS(10, APP_TIMER_PRESCALER)        /**< SPI read interval (ticks). */

#define SECOND_10_MS_UNITS                   100                                             /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(20, UNIT_1_25_MS)                 /**< Minimum connection interval (100 ms). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(40, UNIT_1_25_MS)                 /**< Maximum connection interval (250 ms). */
#define SLAVE_LATENCY                        0                                               /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     (4 * SECOND_10_MS_UNITS)                        /**< Connection supervisory time-out (4 seconds). Supervision time-out uses 10 ms units. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)     /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                               /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                       1                                               /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                       0                                               /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                   0                                               /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                            /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                               /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                               /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                              /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                      /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SPI_INSTANCE                         0                                               /**< SPI instance index. */


static const nrf_drv_spi_t                   spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);       /**< SPI instance. */
static uint8_t                               m_rx_buf[1];                                    /**< SPI RX buffer. */
static const uint8_t                         m_length = sizeof(m_rx_buf);                    /**< SPI RX buffer length. */
static uint8_t                               m_gnss_buffer[256];                             /**< GNSS RX buffer. */
static uint8_t                               m_gnss_index = 0;                               /**< GNSS buffer index. */
static bool                                  m_awaiting_new_message = true;                  /**< Awaiting new GNSS message flag. */
static bool                                  m_location_speed_data_valid = false;            /**< Flag indicating if received gnss data are valid. */
static bool                                  m_gnss_fix = false;                             /**< GNSS Fix. */

static char                                  m_device_name[16];

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;        /**< Handle of the current connection. */
static ble_lns_t                             m_lns;                                          /**< Structure used to identify the location and navigation service. */

APP_TIMER_DEF(m_loc_and_nav_timer_id);                                                       /**< Location and navigation measurement timer. */
APP_TIMER_DEF(m_spi_timer_id);                                                               /**< SPI read timer. */

static ble_lns_loc_speed_t                   m_location_speed;                               /**< Location and speed data. */
static ble_lns_loc_speed_t                   m_last_read_location_speed;                     /**< Last location and speed data read from the  */

static ble_uuid_t                            m_adv_uuids[] = {
                                                               {BLE_UUID_LOCATION_AND_NAVIGATION_SERVICE,  BLE_UUID_TYPE_BLE},
                                                               {BLE_UUID_DEVICE_INFORMATION_SERVICE,       BLE_UUID_TYPE_BLE}
                                                             };

static const ble_lns_loc_speed_t initial_lns_location_speed = {
    .instant_speed_present   = false,
    .total_distance_present  = false,
    .location_present        = true,
    .elevation_present       = false,
    .heading_present         = false,
    .rolling_time_present    = false,
    .utc_time_time_present   = true,
    .position_status         = BLE_LNS_NO_POSITION,
    .data_format             = BLE_LNS_SPEED_DISTANCE_FORMAT_2D,
    .elevation_source        = BLE_LNS_ELEV_SOURCE_POSITIONING_SYSTEM,
    .heading_source          = BLE_LNS_HEADING_SOURCE_COMPASS,
    .latitude                = 0,
    .longitude               = 0,
    .utc_time                = {
                                 .year    = 2017,
                                 .month   = 1,
                                 .day     = 1,
                                 .hours   = 12,
                                 .minutes = 0,
                                 .seconds = 0
                               }
};

static void advertising_start(void);


/**@brief hex to int.
 *
 * @details This function will convert a character hex to int.
 *
 * @param[in] ch  A character in hex.
 *
 * @return  The integer of the hexadecimal character.
 */
uint8_t character_hex_to_int(uint8_t ch) 
{
  if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  else if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;
  else
    return ch - '0';
}


/**@brief Process a GNRMC message.
 */
void process_GNRMC_message(void)
{
    uint8_t message_index, field, field_index;
    int32_t latitude = 0;  //Unit is in degrees with a resolution of 1/(10^7)
    int32_t longitude = 0; //Unit is in degrees with a resolution of 1/(10^7)
    uint8_t status = ' ';
    uint8_t north_south = ' ';
    uint8_t east_west = ' ';
    uint8_t pos_mode = ' ';
    
    //$xxRMC,time,status,lat,NS,long,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs<CR><LF>
    //example: $GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57
    
    for (message_index=0,field=0,field_index=0; (message_index < m_gnss_index) && (field <= 13); message_index++)
    {
        if (m_gnss_buffer[message_index] == ',' || m_gnss_buffer[message_index] == '*')
        {
            field++;
            field_index = 0;
        }
        else
        {
            switch (field) {
                
                case 1: //Time
                    //Hour
                    if (field_index == 0)
                        m_last_read_location_speed.utc_time.hours = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        m_last_read_location_speed.utc_time.hours += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Minute
                    if (field_index == 2)
                        m_last_read_location_speed.utc_time.minutes = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 3)
                        m_last_read_location_speed.utc_time.minutes += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Second
                    if (field_index == 4)
                        m_last_read_location_speed.utc_time.seconds = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 5)
                        m_last_read_location_speed.utc_time.seconds += character_hex_to_int(m_gnss_buffer[message_index]);                    
                    break;
                    
                case 2: //Status
                    status = m_gnss_buffer[message_index];
                    break;
                    
                case 3: //Latitude
                    if (field_index == 0)
                        latitude = 10000000*10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        latitude += 10000000*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 2)
                        latitude += (10000000*10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 3)
                        latitude += (10000000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    //field_index == 4 => '.' -> do nothing
                    if (field_index == 5)
                        latitude += (10000000/10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 6)
                        latitude += (10000000/100*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 7)
                        latitude += (10000000/1000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 8)
                        latitude += (10000000/10000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 9)
                        latitude += (10000000/100000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    break;
                    
                case 4: //North/South
                    north_south = m_gnss_buffer[message_index];
                    break;
                    
                case 5: //Longitude
                    if (field_index == 0)
                        longitude = 10000000*100*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        longitude = 10000000*10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 2)
                        longitude += 10000000*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 3)
                        longitude += (10000000*10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 4)
                        longitude += (10000000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    //field_index == 5 => '.' -> do nothing
                    if (field_index == 6)
                        longitude += (10000000/10*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 7)
                        longitude += (10000000/100*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 8)
                        longitude += (10000000/1000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 9)
                        longitude += (10000000/10000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    if (field_index == 10)
                        longitude += (10000000/100000*character_hex_to_int(m_gnss_buffer[message_index]))/60;
                    break;
                    
                case 6: //East/West
                    east_west = m_gnss_buffer[message_index];
                    break;
                    
                case 9: //Date
                    //Day
                    if (field_index == 0)
                        m_last_read_location_speed.utc_time.day = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 1)
                        m_last_read_location_speed.utc_time.day += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Month
                    if (field_index == 2)
                        m_last_read_location_speed.utc_time.month = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 3)
                        m_last_read_location_speed.utc_time.month += character_hex_to_int(m_gnss_buffer[message_index]);
                    //Year
                    if (field_index == 4)
                        m_last_read_location_speed.utc_time.year = 10*character_hex_to_int(m_gnss_buffer[message_index]);
                    if (field_index == 5)
                        m_last_read_location_speed.utc_time.year += character_hex_to_int(m_gnss_buffer[message_index]);                        
                    break;
                    
                case 12: //posMode
                    pos_mode = m_gnss_buffer[message_index];
                    NRF_LOG_RAW_INFO("posMode\r\n");
                    break;
                    
                default:
                    break;
            }
            field_index++;
        }
    }

    //Data valid
    if (status == 'A')
        m_location_speed_data_valid = true;
    else
        m_location_speed_data_valid = false;

    //GNSS Fix
    if (pos_mode == 'A' || pos_mode == 'D' || pos_mode == 'E' || pos_mode == 'F' || pos_mode == 'R')
        m_gnss_fix = true;
    else
        m_gnss_fix = false;
    
    //Adjust latitude regarding North/South
    if (north_south == 'N')
        m_last_read_location_speed.latitude = latitude;
    else if (north_south == 'S')
        m_last_read_location_speed.latitude = -latitude;
    else 
        m_last_read_location_speed.latitude = 0;

    //Adjust longitude regarding East/West
    if (east_west == 'E')
        m_last_read_location_speed.longitude = longitude;
    else if (east_west == 'W')
        m_last_read_location_speed.longitude = -longitude;
    else 
        m_last_read_location_speed.longitude = 0;
   
    /* Debug */
    NRF_LOG_RAW_INFO("%s\r\n",(uint32_t)m_gnss_buffer);
    NRF_LOG_RAW_INFO("Time: %02d:%02d:%02d\r\n",m_last_read_location_speed.utc_time.hours, 
                     m_last_read_location_speed.utc_time.minutes,m_last_read_location_speed.utc_time.seconds);
    NRF_LOG_RAW_INFO("Date: %02d-%02d-%02d\r\n",m_last_read_location_speed.utc_time.year, 
                     m_last_read_location_speed.utc_time.month,m_last_read_location_speed.utc_time.day);                     
    NRF_LOG_RAW_INFO("Status: %c\r\n",status);
    NRF_LOG_RAW_INFO("pos_mode: %c\r\n",pos_mode);
    NRF_LOG_RAW_INFO("Lat:  %d deg\r\n",m_last_read_location_speed.latitude);
    NRF_LOG_RAW_INFO("Long: %d deg\r\n",m_last_read_location_speed.longitude);
}


/**@brief Process a nmea message.
 */
void process_nmea_message(void)
{
    if ((m_gnss_buffer[1] == 'G') &&
        (m_gnss_buffer[2] == 'N') &&
        (m_gnss_buffer[3] == 'R') && 
        (m_gnss_buffer[4] == 'M') && 
        (m_gnss_buffer[5] == 'C'))
    {
        process_GNRMC_message();
    }

    //NRF_LOG_RAW_INFO("%s",(uint32_t)m_gnss_buffer);
}


/**@brief Function for handling events from the SPI module.
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    if (m_awaiting_new_message == true)
    {
        if (m_rx_buf[0] == '$')
        {
            m_awaiting_new_message = false;
            memset(m_gnss_buffer, 0, sizeof(m_gnss_buffer));
            m_gnss_index = 0;
        }        
    }
    if (m_awaiting_new_message == false)
    {
        if (m_gnss_index >= 255)
        {
            m_awaiting_new_message = true;
        }
        else if (m_rx_buf[0] == '\n')
        {
            process_nmea_message();
            m_awaiting_new_message = true;            
        }
        else
        {
            m_gnss_buffer[m_gnss_index++] = m_rx_buf[0];
        }
    }
}


/**@brief SPI time-out handler.
 *
 * @details This function will be called each time the SPI timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the time-out handler.
 */
 static void spi_timeout_handler(void * p_context)
{    
    //Reset rx buffer
    memset(m_rx_buf, 0, m_length);

    //Read SPI
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, NULL, 0, m_rx_buf, m_length));
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Callback function for errors in the Location Navigation Service.
 *
 * @details This function will be called in case of an error in the Location Navigation Service.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 */
static void lns_error_handler(uint32_t err_code)
{
    app_error_handler(DEAD_BEEF, 0, 0);
}


/**@brief Location Navigation event handler.
 *
 * @details This function will be called for all events of the Location Navigation Module that
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Location Navigation Module.
 */
static void on_lns_evt(ble_lns_t const * p_lns, ble_lns_evt_t const * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_LNS_LOC_SPEED_EVT_NOTIFICATION_ENABLED:
            NRF_LOG_INFO("Location/Speed: Notification enabled\r\n");
            break;

        case BLE_LNS_LOC_SPEED_EVT_NOTIFICATION_DISABLED:
            NRF_LOG_INFO("Location/Speed: Notification disabled\r\n");
            break;

        default:
            break;
    }
}


/**@brief Location and navigation time-out handler.
 *
 * @details This function will be called each time the location and navigation measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the time-out handler.
 */
static void loc_and_nav_timeout_handler(void * p_context)
{
    uint32_t err_code;

    UNUSED_PARAMETER(p_context);

    if (m_location_speed_data_valid == true)
    {
        m_location_speed.latitude = m_last_read_location_speed.latitude;
        m_location_speed.longitude = m_last_read_location_speed.longitude;
        m_location_speed.utc_time = m_last_read_location_speed.utc_time;
        
        err_code = ble_lns_loc_speed_send(&m_lns);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}


/**@brief Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&m_loc_and_nav_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                loc_and_nav_timeout_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_spi_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                spi_timeout_handler);
    APP_ERROR_CHECK(err_code);    
}


/**@brief Convert hex number to ascii character.
 */
static char get_char_from_nibble(uint8_t n)
{
    if (n > 9)
        return 'A' + n - 10;
	else
        return '0' + n;
}


/**@brief GAP initialization.
 *
 * @details This function shall be used to set up all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_addr_t          addr;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    size_t mac_start = strlen(DEVICE_NAME_PREFIX);
    strcpy(m_device_name, DEVICE_NAME_PREFIX);

    // Append the last four nibbles of the MAC address to the device name
    m_device_name[mac_start++] = get_char_from_nibble((NRF_FICR->DEVICEADDR[0] >> 12) & 0xF);
    m_device_name[mac_start++] = get_char_from_nibble((NRF_FICR->DEVICEADDR[0] >>  8) & 0xF);
    m_device_name[mac_start++] = get_char_from_nibble((NRF_FICR->DEVICEADDR[0] >>  4) & 0xF);
    m_device_name[mac_start++] = get_char_from_nibble( NRF_FICR->DEVICEADDR[0]        & 0xF);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_device_name,
                                          strlen(m_device_name));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_DISP);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    
    // Change to public address
    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    err_code = sd_ble_gap_addr_set(&addr);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize services that will be used by the application.
 *
 * @details Initialize the Location and Navigation, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_lns_init_t lns_init;
    ble_dis_init_t dis_init;

    memset(&lns_init, 0, sizeof(lns_init));

    lns_init.evt_handler        = on_lns_evt;
    lns_init.error_handler      = lns_error_handler;

    lns_init.is_position_quality_present = false;
    lns_init.is_control_point_present    = false;
    lns_init.is_navigation_present       = false;

    lns_init.available_features     = BLE_LNS_FEATURE_LOCATION_SUPPORTED |
                                      BLE_LNS_FEATURE_UTC_TIME_SUPPORTED;


    m_location_speed   = initial_lns_location_speed;

    lns_init.p_location_speed   = &m_location_speed;

    lns_init.loc_nav_feature_security_req_read_perm  = SEC_OPEN;
    lns_init.loc_speed_security_req_cccd_write_perm  = SEC_OPEN;

    err_code = ble_lns_init(&m_lns, &lns_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUMBER);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, SW_VERSION);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_loc_and_nav_timer_id, LOC_AND_NAV_DATA_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_spi_timer_id, SPI_READ_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);    
}


/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_lns.loc_speed_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    //Set GNSS and Acceleromter power to OFF
    nrf_gpio_pin_write(4,0);    
    
    // Prepare wakeup button.
    err_code = bsp_wakeup_button_enable(0); //ON/OFF button
    APP_ERROR_CHECK(err_code);
    err_code = bsp_wakeup_button_disable(1); //GPS_EN button
    APP_ERROR_CHECK(err_code);
    
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events that are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW); //BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED
        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_lns_on_ble_evt(&m_lns, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;
            
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond              = SEC_PARAM_BOND;
    sec_param.mitm              = SEC_PARAM_MITM;
    sec_param.lesc              = SEC_PARAM_LESC;
    sec_param.keypress          = SEC_PARAM_KEYPRESS;
    sec_param.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob               = SEC_PARAM_OOB;
    sec_param.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc     = 1;
    sec_param.kdist_own.id      = 1;
    sec_param.kdist_peer.enc    = 1;
    sec_param.kdist_peer.id     = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Power to GNSS module and Accelerometer initialization.
 */
static void gnss_accel_power_init(void)
{
    //Set pin SW_EN to output pin
    nrf_gpio_cfg_output(4);
    
    //Set power ON
    nrf_gpio_pin_write(4,1);
}


/**@brief SPI functionality initialization.
 */
static void spi_init(void)
{
    nrf_drv_spi_config_t spi_config;

    //Configure and initiate SPI
    spi_config.ss_pin       = SPI_SS_PIN;
    spi_config.miso_pin     = SPI_MISO_PIN;
    spi_config.mosi_pin     = SPI_MOSI_PIN;
    spi_config.sck_pin      = SPI_SCK_PIN;
    spi_config.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;
    spi_config.orc          = 0xFF;  //Over-run character. This character is used when all bytes from the TX buffer are sent, but the transfer continues due to RX.
    spi_config.frequency    = NRF_DRV_SPI_FREQ_4M;
    spi_config.mode         = NRF_DRV_SPI_MODE_0;
    spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
}

/**@brief Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize
    timers_init();
    buttons_leds_init(&erase_bonds);

    ble_stack_init();
    peer_manager_init(erase_bonds);
    if (erase_bonds == true)
    {
        NRF_LOG_INFO("Bonds erased!\r\n");
    }
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    spi_init();
    gnss_accel_power_init();    
    
    // Start execution
    application_timers_start();
    NRF_LOG_INFO("Location and Navigation App started\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
