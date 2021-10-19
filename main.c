/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// For the LBS
#include "boards.h"
#include "app_button.h"
#include "ble_lbs.h"

// Benji's imports
#include "app_util_platform.h"
#include "nrfx_twim.h"
#include "MAX32644.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

/*Define MFIO and RSTN pins*/

#define SDA_PIN 11 //23
#define SCL_PIN 12 //22
#define RSTN_PIN 13 //25 //20
#define MFIO_PIN 14 //24 
/* TWI instance ID. */
#define TWIM_INSTANCE_ID     0

 /* Number of possible TWI addresses. */
 #define TWIM_ADDRESSES      127

/* TWI instance. */
static const nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(TWIM_INSTANCE_ID);
uint8_t address = 0x55;
uint8_t tx_data[255];
uint8_t rx_data[255];
uint8_t status;

#define DEVICE_NAME                         "UpBeat-BT-HR"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(1000)                   /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      1                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define LEDBUTTON_LED                       BSP_BOARD_LED_4
#define LEDBUTTON_BUTTON                    BSP_BUTTON_0
#define BUTTON_DETECTION_DELAY              APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */



BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_LBS_DEF(m_lbs);                                                 /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_heart_rate_timer_id);                               /**< Heart rate measurement timer. */

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool     m_rr_interval_enabled = true;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static  uint16_t dummy = 110;
static uint8_t zone_val = 0;
static uint16_t hrVal = 0;
static uint8_t algoStateGlobal = 0;
static uint16_t storageHR = 0;

/*
 * Benji's Functions Start
*/

/**
 * @brief TWI initialization.
 */
void twim_init (void)
{
    ret_code_t err_code;
    
    const nrfx_twim_config_t twim_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = (nrf_twim_frequency_t)NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY,
       .interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
       .hold_bus_uninit    = NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT
    };

    err_code = nrfx_twim_init(&m_twim, &twim_config, NULL, NULL); //MUST INITIALIZE
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twim); //MUST ENABLE
}


void init_MAX32664(void){
  ret_code_t err_code;
  nrf_gpio_cfg_output(RSTN_PIN);
  nrf_gpio_cfg_output(MFIO_PIN);

  nrf_gpio_pin_clear(RSTN_PIN); //setting RSTN low for 10ms
  nrf_delay_us(5);
  nrf_gpio_pin_set(MFIO_PIN); //setting MFIO to high
  nrf_delay_us(5);

  nrf_gpio_pin_set(RSTN_PIN);
  nrf_delay_us(50);

  nrf_delay_us(2000);

  //READ BIOHUB MODE (ALGORITHIM|BOOTLOADER)

  tx_data[0] = 0x02;
  tx_data[1] = 0x00;
  
  nrfx_twim_xfer_desc_t TX2Bytes = NRFX_TWIM_XFER_DESC_TX(address, (uint8_t*)tx_data, sizeof(uint8_t)*2);
  nrfx_twim_xfer_desc_t STATUS = NRFX_TWIM_XFER_DESC_RX(address, &status, sizeof(uint8_t)); 
  err_code = nrfx_twim_xfer(&m_twim, &TX2Bytes, 0);
    nrf_delay_ms(30);
  err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
  printf("%d", status);
  if(status == 0){
     printf("Application mode");
  }


}

void config_MAX32664(void){
    ret_code_t err_code;
    nrfx_twim_xfer_desc_t TX3Bytes = NRFX_TWIM_XFER_DESC_TX(address, (uint8_t*)tx_data, sizeof(uint8_t)*3);
    nrfx_twim_xfer_desc_t TX2Bytes = NRFX_TWIM_XFER_DESC_TX(address, (uint8_t*)tx_data, sizeof(uint8_t)*2);
    nrfx_twim_xfer_desc_t STATUS = NRFX_TWIM_XFER_DESC_RX(address, &status, sizeof(uint8_t));
    nrfx_twim_xfer_desc_t RX2Bytes = NRFX_TWIM_XFER_DESC_RX(address, &rx_data, sizeof(uint8_t)*2);
    
    
    //CALIBRATE HUB
    uint8_t tx_data1[14] = {0x50, 0x02, 0x0b, 0x00, 0x02, 0x6f, 0x60, 0xff, 0xcb, 0x1d, 0x12, 0x00, 0xab, 0xf3, 0x7b};
    nrfx_twim_xfer_desc_t CALIBRATE = NRFX_TWIM_XFER_DESC_TX(address, (uint8_t*)tx_data1, sizeof(uint8_t)*15);

    err_code = nrfx_twim_xfer(&m_twim, &CALIBRATE, 0);
    nrf_delay_ms(30);
    err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    if(status != 0){
     printf("1. Error in config\n%04X\n", status);
    }

    //SET OUTPUTMODE TO SENSOR+ALGORITHM DATA
    tx_data[0] = SET_OUTPUT_MODE;
    tx_data[1] = 0x00;
    tx_data[2] = 0x02;

    err_code = nrfx_twim_xfer(&m_twim, &TX3Bytes, 0);
    nrf_delay_ms(30);
    err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    if(status != 0){
     printf("1. Error in config\n%04X\n", status);
    }

    
    //SETTING INTERRUPT THERSHOLD
    tx_data[0] = SET_OUTPUT_MODE;
    tx_data[1] = 0x01;
    tx_data[2] = 0x0f;

    err_code = nrfx_twim_xfer(&m_twim, &TX3Bytes, 0);
    nrf_delay_ms(30);
    err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    if(status != 0){
     printf("1. Error in config\n%04X\n", status);
    }

    //ENABLE AGC ALGORITHM
    tx_data[0] = ALGORITHM_MODE_ENABLE;
    tx_data[1] = 0x00;
    tx_data[2] = 0x01;
   
    err_code = nrfx_twim_xfer(&m_twim, &TX3Bytes, 0);
    nrf_delay_ms(30);
    err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    if(status != 0){
     printf("2. Error in config\n%04X\n", status);
    }

    //ENABLE ACCELEROMETER
    //tx_data[0] = SENSOR_MODE_ENABLE;
    //tx_data[1] = 0x04;
    //tx_data[2] = 0x01;
    //tx_data[3] = 0x00;
    //nrfx_twim_xfer_desc_t TX4Bytes = NRFX_TWIM_XFER_DESC_TX(address, (uint8_t*)tx_data, sizeof(uint8_t)*4);

    //err_code = nrfx_twim_xfer(&m_twim, &TX4Bytes, 0);
    //nrf_delay_ms(30);
    //err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    //if(status != 0){
    // printf("3. Error in config\n%04X\n", status);
    //}

    //ENABLE MAX30101 SENSOR
    tx_data[0] = SENSOR_MODE_ENABLE;
    tx_data[1] = 0x03;
    tx_data[2] = 0x01;

    err_code = nrfx_twim_xfer(&m_twim, &TX3Bytes, 0);
    nrf_delay_ms(50);
    err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    if(status != 0){
     printf("4. Error in config\n%04X\n", status);
    }

    //ENABLE MAXIMFAST ALGORITHM MODE 1
    tx_data[0] = ALGORITHM_MODE_ENABLE;
    tx_data[1] = 0x02;
    tx_data[2] = 0x01; //CHANGE TO MODE 1

    err_code = nrfx_twim_xfer(&m_twim, &TX3Bytes, 0);
    nrf_delay_ms(50);
    err_code = nrfx_twim_xfer(&m_twim, &STATUS, 0);
    if(status != 0){
     printf("5. Error in config\n%04X\n", status);
    }

    nrf_delay_ms(100);

    //READ CONFIGURATION
    tx_data[0] = 0x11;
    tx_data[1] = 0x00;
    err_code = nrfx_twim_xfer(&m_twim, &TX2Bytes, 0);
    nrf_delay_ms(50);
    err_code = nrfx_twim_xfer(&m_twim, &RX2Bytes, 0);
}

/*
 * Benji's Functions End
*/


static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,              BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE,   BLE_UUID_TYPE_BLE}
};


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


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

//static uint16_t getHR(uint16_t * dummy) 
//{
//  *dummy += 1;
//  return (*dummy * 10 + 5) % 100;
//  //return *dummy;
//}
/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */

static void heart_rate_meas_timeout_handler(void * p_context)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(p_context);

    //heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);
    heart_rate = storageHR;

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT); // can be changed to BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON:
            NRF_LOG_INFO("Send button state change.");
            if (button_action == 0) return;
            zone_val++;
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action, zone_val);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED);
        NRF_LOG_INFO("Received LED OFF!");
    }
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_bas_init_t     bas_init;
    ble_lbs_init_t     init = {0};
    //ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
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
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
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


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
        
        case BSP_EVENT_KEY_0:
            printf("helloo");
            NRF_LOG_INFO("Send button state change.");
            uint8_t button_action = 1;
            zone_val++;
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action, zone_val);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;   

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void handleHR(uint16_t * storage, uint16_t * hr)
{
  if (algoStateGlobal == 1)
  {
    if (*hr != 0)
    {
      *storage = *hr;
    }
  }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    ret_code_t err_code;

    // Initialize.
    log_init();
    timers_init();
    //buttons_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    //sensor_simulator_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("Heart Rate Sensor example started.");
    application_timers_start();
    advertising_start(erase_bonds);

    // Benji's stuff
    
    //APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    //NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("TWI scanner started.");
    NRF_LOG_FLUSH();
    twim_init();

    init_MAX32664();
    config_MAX32664();
    
    //READ BIOMETRIC HUB STATUS
    tx_data[0] = READ_SENSOR_HUB_STATUS;
    tx_data[1] = 0x00;
    nrfx_twim_xfer_desc_t TX2Bytes = NRFX_TWIM_XFER_DESC_TX(address, (uint8_t*)tx_data, sizeof(uint8_t)*2);
    nrfx_twim_xfer_desc_t RX2Bytes = NRFX_TWIM_XFER_DESC_RX(address, (uint8_t*)rx_data, sizeof(uint8_t)*2);
    //int dataRdy = 0;
    //while (dataRdy != 8) {
    //  err_code = nrfx_twim_xfer(&m_twim, &TX2Bytes, 0);
    //  nrf_delay_ms(3);    
    //  err_code = nrfx_twim_xfer(&m_twim, &RX2Bytes, 0);
    //  dataRdy = rx_data[1] & (1 << 3);
    //  printf("0-%d\n", rx_data[0]);
    //  printf("1-%d\n", rx_data[1]);
    //  printf("2-%d\n", rx_data[2]);
    //  printf("3-%d\n", rx_data[3]);
    //  printf("%d", dataRdy);
    //}



    err_code = nrfx_twim_xfer(&m_twim, &TX2Bytes, 0);
    nrf_delay_ms(3);    
    err_code = nrfx_twim_xfer(&m_twim, &RX2Bytes, 0);

    //READ NUMBER OF SAMPLES IN FIFO
    tx_data[0] = READ_OUTPUT_FIFO;
    tx_data[1] = 0x00;
    
    err_code = nrfx_twim_xfer(&m_twim, &TX2Bytes, 0);
    nrf_delay_ms(30);
    err_code = nrfx_twim_xfer(&m_twim, &RX2Bytes, 0);

    int numSamples = rx_data[1];

    //READ DATA STORED IN FIFO
    tx_data[0] = READ_OUTPUT_FIFO;
    tx_data[1] = 0x01;
    nrfx_twim_xfer_desc_t RX25Bytes = NRFX_TWIM_XFER_DESC_RX(address, (uint8_t*)rx_data, sizeof(uint8_t)*26);

    

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();

        err_code = nrfx_twim_xfer(&m_twim, &TX2Bytes, 0);
        nrf_delay_ms(10);
        err_code = nrfx_twim_xfer(&m_twim, &RX25Bytes, 0);

        uint16_t num = (rx_data[19] << 8);
        num |= rx_data[20];
        hrVal= num/10;
        uint16_t algoState = rx_data[24]; // 0x03 HR Detected
        if (algoState == 0X03) { //First HR reading is done 
          algoStateGlobal = 1;
        }
        handleHR(&storageHR, &hrVal);
        status = rx_data[25];

        printf("%d\n", storageHR);

      
        NRF_LOG_INFO("HR: %d\n",storageHR);
        NRF_LOG_INFO("Confidence: %d\n\n", rx_data[21]);
        NRF_LOG_FLUSH();
        nrf_delay_ms(1000);   
    }

}


