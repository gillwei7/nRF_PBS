/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_srv_dis Device Information Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Device Information Service module.
 *
 * @details This module implements the Device Information Service.
 *          During initialization it adds the Device Information Service to the BLE stack database.
 *          It then encodes the supplied information, and adds the curresponding characteristics.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_GILL_H__
#define BLE_GILL_H__

#include <stdint.h>
#include "ble_srv_common.h"


// Gill add


typedef struct
{
	// Value Exisence Flag
	uint8_t flag;
	// Value Setting 
	uint8_t sampling_interval_value;
	uint8_t led_blinking_duration;
	int16_t small_accident_value;
	int16_t medium_accident_level;
	int16_t high_accident_level;
	int16_t hard_accelaration_level;
	int16_t hard_braking_level;
	int16_t hard_steering_level;
	uint32_t current_utc;
}bsc_t;

typedef struct
{
	uint8_t download_control_point;
	uint16_t number_of_event;
}esc_t;

typedef struct
{
	uint8_t recorded_data_interval;
	uint8_t recorded_data_resolution;
	uint8_t recorded_number_of_axis;
	uint8_t recorded_event_id;
	uint8_t	recorded_event_duration;
	uint32_t recorded_utc_of_event_start;
}drhc_t;

typedef struct
{
	uint8_t data_packet_id;
	uint8_t data_packet_length;
	uint8_t *data_payload;
}cdrc_t;

typedef struct
{
	uint8_t data_packet_id;
	uint8_t data_packet_length;
	uint8_t *data_payload;
}rdrc_t;

typedef enum
{
    BLE_PBS_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    BLE_PBS_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} ble_pbs_evt_type_t;

/**@brief PBS event. */
typedef struct
{
    ble_pbs_evt_type_t evt_type;                                  /**< Type of event. */
} ble_pbs_evt_t;


// Forward declaration of the ble_bas_t type. 
typedef struct ble_pbs_s ble_pbs_t;

/**@brief PBS event handler type. */
typedef void (*ble_pbs_evt_handler_t) (ble_pbs_t * p_pbs, ble_pbs_evt_t * p_evt);

/**@brief Pioneer Beacon Service init structure. This contains all possible characteristics 
 *        needed for initialization of the service.
 */
typedef struct
{
	bsc_t bsc_s;
	esc_t esc_s;
	drhc_t drhc_s;
	cdrc_t cdrc_s;
	rdrc_t rdrc_s;
	 // Security configuration
	ble_srv_security_mode_t    pbs_attr_md; 
	ble_srv_cccd_security_mode_t pbs_cccd_md;
	//Event handle
	ble_pbs_evt_handler_t         evt_handler;  // Detect for there's a event or not
} ble_pbs_init_t;

struct ble_pbs_s
{
  // All Data
	bsc_t bsc_s;
	esc_t esc_s;
	drhc_t drhc_s;
	cdrc_t cdrc_s;
	rdrc_t rdrc_s;
	 // Security configuration
	ble_srv_security_mode_t    pbs_attr_md; 
	ble_srv_cccd_security_mode_t pbs_cccd_md;
	uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
	ble_gatts_char_handles_t      bsc_handles;
	ble_gatts_char_handles_t      esc_handles;                      /**<Event Storage Characteristic handle>**/
	ble_gatts_char_handles_t      drhc_handles;
	ble_gatts_char_handles_t      cdrc_handles;
	ble_gatts_char_handles_t      rdrc_handles;
	// Event handle
	ble_pbs_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
	bool is_notification_supported;  // No use
};

/**@brief Function for initializing the Pioneer Beacon Service.
 *
 * @details This call allows the application to initialize the device information service. 
 *          It adds the DIS service and DIS characteristics to the database, using the initial
 *          values supplied through the p_dis_init parameter. Characteristics which are not to be
 *          added, shall be set to NULL in p_dis_init.
 *
 * @param[in]   p_pbs_init   The structure containing the values of characteristics needed by the
 *                           service.
 *
 * @return      NRF_SUCCESS on successful initialization of service.
 */
uint32_t ble_pbs_init(ble_pbs_t * p_pbs);

void ble_bas_on_ble_evt(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt);

uint32_t ble_pbs_dhrc_update(ble_pbs_t * p_pbs, uint8_t dhrc_data);
void ble_pbs_on_ble_evt(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt);

uint32_t ble_pbs_cdrc_update(ble_pbs_t * p_pbs, uint8_t *cdrc_data);

#endif // BLE_GILL_H__

/** @} */
