/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
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
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "bsp_btn_ble.h"
#include "ble_pbs.h"
#include "app_uart.h"
// include for int_flash.c
//#include "math.h"
#include "int_flash.h"
//#include "ble_gap.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "Gill_BLE"                               /**< Name of device. Will be included in the advertising data. */


// Gill add
/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */
#define DATA_HEADER_REPORT_INTERVAL      APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 
#define MIN_DRHC_DATA                		 1                                         
#define MAX_DRHC_DATA               	 	 32                                        
#define DRHC_INCREMENT          				 1 

#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(400, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(650, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_pbs_t                         m_pbs;                                     /**< Structure used to identify the PBS. */

static sensorsim_cfg_t                   m_drhc_sim_cfg;                         
static sensorsim_state_t                 m_drhc_sim_state;

APP_TIMER_DEF(m_data_header_report_id);  

#define APP_ADV_FAST_INTERVAL            0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL            0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT             30                                          /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT             180                                         /**< The duration of the slow advertising period (in seconds). */

//#define DEBUG													 1 

#define DEFAULT_SAMPLING_FREQENCY 			 50
#define DEFAULT_RECORD_DURATION 				 10
static uint8_t 													 counter; // assigned in main()

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
//                                   {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
//                                   {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};



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


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
//static void ble_drhc_update(void)
//{
//    uint32_t err_code;
//    uint8_t  drhc_sim_data;

//    drhc_sim_data = (uint8_t)sensorsim_measure(&m_drhc_sim_state, &m_drhc_sim_cfg);

//    err_code = ble_pbs_drhc_update(&m_pbs, drhc_sim_data);
//    if ((err_code != NRF_SUCCESS) &&
//        (err_code != NRF_ERROR_INVALID_STATE) &&
//        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
//        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//        )
//    {
//        APP_ERROR_HANDLER(err_code);
//    }
//}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void battery_level_meas_timeout_handler(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);
//    battery_level_update();
//}
static void dhrc_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);
		uint32_t r_xy;
		uint32_t r_z;
		uint8_t r_event_ID;
		uint32_t r_UTC;
		uint32_t err_code;
		int ret;
		uint16_t data_x[4] = {0};
		uint16_t data_y[4] = {0};
		uint8_t len = 0;
		uint8_t p_cal_encoded_buffer[18];
		
		if (counter >3)
		{
			counter = counter-4;
			// Simulator flash read
			for(int i=0;i<4;i++)
			{
				ret=flash_data_set_read(NULL, NULL, &r_xy, &r_z, &r_event_ID, &r_UTC);
				data_x[i] = (uint16_t)((r_xy&0xFFFF0000)>>16)/100-16;
				data_y[i] = (uint16_t)((r_xy&0x0000FFFF)>>16)/100-16;
				if (ret < 0)	
					printf("ret error:0\r\n");
				else
					printf("ret: %i read x: %f read y: %f read z: %f read_eventID: %i read UTC: %u \n\r", ret,(float)((r_xy&0xFFFF0000)>>16)/100-16,(float)(r_xy&0x0000FFFF)/100-16, (float)r_z/100-16, r_event_ID, r_UTC);
				
				nrf_delay_ms(10);
			}
		}
    //uint8_t  cal_sim_data[18] = {0};
		
		
		//err_code = ble_pbs_cal_update(&m_pbs, cal_sim_data);
		//for (int i=0;i<18;i++)
			//p_cal_encoded_buffer[len++] = ;
//	// Check Download control point
//    ble_gatts_value_t gatts_value;
//		memset(&gatts_value, 0, sizeof(gatts_value));
//		gatts_value.len     = 1;
//		gatts_value.offset  = 0;
//		//gatts_value.p_value = &dhrc_data;
//		err_code = sd_ble_gatts_value_get(m_pbs.conn_handle,
//																			m_pbs.esc_handles.value_handle,
//																			&gatts_value);
//	printf("get value err:%04X, value:%02X\r\n",err_code,gatts_value.p_value[0]);
	
//    uint8_t  drhc_sim_data;
//    drhc_sim_data = (uint8_t)sensorsim_measure(&m_drhc_sim_state, &m_drhc_sim_cfg);
//    err_code = ble_pbs_drhc_update(&m_pbs, drhc_sim_data);
		
		
		
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
	
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
	    err_code = app_timer_create(&m_data_header_report_id,
                                APP_TIMER_MODE_REPEATED,
                                dhrc_timeout_handler);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
//		ble_pbs_init_t pbs_init; //No use currently

		m_pbs.evt_handler          = NULL;
		memset(&m_pbs, 0, sizeof(m_pbs));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_pbs.pbs_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_pbs.pbs_attr_md.write_perm); // Orignally no access
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_pbs.pbs_cccd_md.cccd_write_perm);
		
		// Value setting 
		//bool utc_exist_bit = false;  // bit located in m_pbs.bsc_s.flag 0x10, TBD
		m_pbs.bsc_s.flag = 0x00;
		m_pbs.bsc_s.sampling_frequency = 50;
		m_pbs.bsc_s.ble_output_power = -8;
		m_pbs.bsc_s.small_accident_level_x = 70;
		m_pbs.bsc_s.small_accident_level_y = 60;
		m_pbs.bsc_s.medium_accident_level = 40;
		m_pbs.bsc_s.high_accident_level = 150;
		m_pbs.bsc_s.hard_accelaration_level = -30;
		m_pbs.bsc_s.hard_braking_level = 35;
		m_pbs.bsc_s.hard_steering_level_left = 35;
		m_pbs.bsc_s.hard_steering_level_right= 35;
		m_pbs.bsc_s.current_utc = 0;
		//Beacon Status Report default value
		m_pbs.bsrc_s.acc_voltage = 0;
		m_pbs.bsrc_s.ambient_sensor_value = 0;
		m_pbs.bsrc_s.button_status = 0;
		// Event Storage Characteristic default value
		m_pbs.esc_s.download_control_point = 0x00;
		m_pbs.esc_s.number_of_event = 0x00;
		// Data Report Header Characteristic default value
		m_pbs.drhc_s.recorded_data_interval = 20;
		m_pbs.drhc_s.recorded_data_resolution = 12;
		m_pbs.drhc_s.recorded_number_of_axis = 3;
		m_pbs.drhc_s.recorded_event_id = 0;
		m_pbs.drhc_s.recorded_event_duration = 10;
		m_pbs.drhc_s.recorded_utc_of_event_start = 0;
		// Calibration Data Log default value
		m_pbs.cdrc_s.data_packet_id = 0;
		m_pbs.cdrc_s.data_packet_length = 18;
		uint8_t default_cdrc_data[18] = {0};
		m_pbs.cdrc_s.data_payload = default_cdrc_data;
		// Raw Data Log default value
		m_pbs.rdrc_s.data_packet_id = 0;
		m_pbs.rdrc_s.data_packet_length = 18;
		uint8_t default_rdrc_data[18] = {0};
		m_pbs.rdrc_s.data_payload = default_rdrc_data;
		
		
    err_code = ble_pbs_init(&m_pbs);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
	  m_drhc_sim_cfg.min          = MIN_DRHC_DATA;
    m_drhc_sim_cfg.max          = MAX_DRHC_DATA;
    m_drhc_sim_cfg.incr         = DRHC_INCREMENT;
    m_drhc_sim_cfg.start_at_max = true;

    sensorsim_init(&m_drhc_sim_state, &m_drhc_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
		err_code = app_timer_start(m_data_header_report_id, DATA_HEADER_REPORT_INTERVAL, NULL);
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
    uint32_t err_code;

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
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
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
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
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



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            break;
//				case BLE_GATTS_EVT_WRITE:
//						printf("BLE_GATTS_EVT_WRITE\r\n");
//					 // do things in ble_pbs_on_ble_evt which call from softdevice handler
//						break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
		ble_pbs_on_ble_evt(&m_pbs, p_ble_evt);
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
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

//#ifdef BLE_DFU_APP_SUPPORT
//    ble_enable_params.gatts_enable_params.service_changed = 1;
//#endif // BLE_DFU_APP_SUPPORT
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
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

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
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
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


// Gill add 
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

//static void advertising_init(void)
//{
//    uint32_t       err_code;
//    uint8_t        adv_flags;
//    ble_advdata_t  advdata;

//    // Build and set advertising data
//    memset(&advdata, 0, sizeof(advdata));

//    adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
//    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance      = true;
//    advdata.flags                   = adv_flags;
//    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    advdata.uuids_complete.p_uuids  = m_adv_uuids;

//    ble_adv_modes_config_t options =
//    {
//        BLE_ADV_WHITELIST_ENABLED,
//        BLE_ADV_DIRECTED_ENABLED, 
//        BLE_ADV_DIRECTED_SLOW_DISABLED, 0,0,
//        BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
//        BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT
//    };

//    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, ble_advertising_error_handler);
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for initializing buttons and leds.
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


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief UART events handler.
 */
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**
 * @brief UART initialization.
 */
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
		uart_config();
    app_trace_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
	
		// Start flash write/read simulation
		float cal_acc_test[3];	
		cal_acc_test[0] = 1.23;
		cal_acc_test[1] = 15.67;
		cal_acc_test[2] = -15.67;			
		int w_cnt = 600 * 3;
		flash_data_set_init();	
		while(w_cnt--) {
				if (w_cnt == 1000) {
					flash_data_set_write(NULL, cal_acc_test, 1, w_cnt);
				} else if (w_cnt == 500) {
					flash_data_set_write(NULL, cal_acc_test, 2, w_cnt);
				}	else {
					flash_data_set_write(NULL, cal_acc_test,0, 0);
				}	
		}	
		counter = DEFAULT_SAMPLING_FREQENCY*DEFAULT_RECORD_DURATION/4;
//		//tx power set test
//		uint32_t tx_power = 4;
//		sd_ble_gap_tx_power_set(tx_power);

    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
#if PBS_DEBUG
		printf("ADV start\r\n");
#endif
    // Enter main loop.
    for (;;)
    {			
        power_manage();
    }
}
