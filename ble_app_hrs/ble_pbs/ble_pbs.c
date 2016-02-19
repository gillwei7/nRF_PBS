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

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

/**
 * File: ble_pbs.c
 * Description: BLE Pioneer Beacon Service 
 *
 * Copyright 2016 by CYNTEC Corporation.  All rights reserved.
 * First Author: Gill Wei
 */


#include "ble_pbs.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"

//15422fe0-bce5-11e5-a837-0800200c9a66
#define PBS_UUID_BASE {0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x37, 0xA8, 0xE5, 0x11, 0xE5, 0xBC, 0xE0, 0x2F, 0x42, 0x15}
#define PBS_SERVICE_SHORT_UUID 										0x2FE0
#define PBS_BEACON_SETTING_CHAR_SHORT_UUID 				0x2FE1
#define PBS_EVENT_STORAGE_CHAR_SHORT_UUID 				0x2FE2
#define PBS_DATA_REPORT_HEADER_CHAR_SHORT_UUID 		0x2FE3
#define PBS_CAL_DATA_REPORT_CHAR_SHORT_UUID 			0x2FE4
#define PBS_RAW_DATA_REPORT_CHAR_SHORT_UUID 			0x2FE5

#define BSC_ALLDATA_LENGTH														19
#define ESC_ALLDATA_LENGTH														 3
#define DRHC_ALLDATA_LENGTH														 9
#define CDRC_ALLDATA_MIN_LENGTH												 3
#define CDRC_ALLDATA_MAX_LENGTH												20
#define RDRC_ALLDATA_MIN_LENGTH												 3
#define RDRC_ALLDATA_MAX_LENGTH												20

#define PBS_DEBUG																			 1

static uint16_t                 service_handle;
//static ble_gatts_char_handles_t beacon_setting_handles;
//static ble_gatts_char_handles_t event_storage_handles;
//static ble_gatts_char_handles_t data_report_header_handles;
//static ble_gatts_char_handles_t cal_data_report_handles;
//static ble_gatts_char_handles_t raw_data_report_handles;

static bool esc_notify_flag;
static bool drhc_notify_flag;
static bool cdrc_notify_flag;
static bool rdrc_notify_flag;

// Encode All Characteristics' Data
static void bsc_data_encode(uint8_t * p_encoded_buffer, const bsc_t * p_bsc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_bsc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_bsc->flag;
		p_encoded_buffer[len++] = p_bsc->sampling_interval_value;
		p_encoded_buffer[len++] = p_bsc->led_blinking_duration;
		len += uint16_encode(p_bsc->small_accident_value, &p_encoded_buffer[len]);
		len += uint16_encode(p_bsc->medium_accident_level, &p_encoded_buffer[len]);
		len += uint16_encode(p_bsc->high_accident_level, &p_encoded_buffer[len]);
		len += uint16_encode(p_bsc->hard_accelaration_level, &p_encoded_buffer[len]);
		len += uint16_encode(p_bsc->hard_braking_level, &p_encoded_buffer[len]);
		len += uint16_encode(p_bsc->hard_steering_level, &p_encoded_buffer[len]);
		len += uint32_encode(p_bsc->current_utc, &p_encoded_buffer[len]);
}
static void esc_data_encode(uint8_t * p_encoded_buffer, const esc_t * p_esc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_esc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_esc->download_control_point;
		len += uint16_encode(p_esc->number_of_event, &p_encoded_buffer[len]);
}
static void drhc_data_encode(uint8_t * p_encoded_buffer, const drhc_t * p_drhc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_drhc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_drhc->recorded_data_interval;
		p_encoded_buffer[len++] = p_drhc->recorded_data_resolution;
		p_encoded_buffer[len++] = p_drhc->recorded_number_of_axis;
		p_encoded_buffer[len++] = p_drhc->recorded_event_id;
		p_encoded_buffer[len++] = p_drhc->recorded_event_duration;
		len += uint32_encode(p_drhc->recorded_utc_of_event_start, &p_encoded_buffer[len]);
}
static void cdrc_data_encode(uint8_t * p_encoded_buffer, const cdrc_t * p_cdrc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_cdrc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_cdrc->data_packet_id;
		p_encoded_buffer[len++] = p_cdrc->data_packet_length;
	for (int i=0;i<p_cdrc->data_packet_length;i++)
		p_encoded_buffer[len++] = p_cdrc->data_payload[i];
}
static void rdrc_data_encode(uint8_t * p_encoded_buffer, const rdrc_t * p_rdrc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_rdrc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_rdrc->data_packet_id;
		p_encoded_buffer[len++] = p_rdrc->data_packet_length;
	for (int i=0;i<p_rdrc->data_packet_length;i++)
		p_encoded_buffer[len++] = p_rdrc->data_payload[i];
}

/**@brief Function for adding the Characteristic.
 *
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[in]   dis_attr_md    Security settings of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t pbs_char_add(const ble_gatts_char_md_t  char_md,
													ble_uuid_t                  char_uuid,
                         uint8_t                       * p_char_value,
                         uint16_t                        char_len,
                         const ble_srv_security_mode_t * pbs_attr_md,
                         ble_gatts_char_handles_t      * p_handles)
{
		char_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
	
    
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
		
	
    APP_ERROR_CHECK_BOOL(p_char_value != NULL);
    APP_ERROR_CHECK_BOOL(char_len > 0);

    

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = pbs_attr_md->read_perm;
    attr_md.write_perm = pbs_attr_md->write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = char_len;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = char_len;
    attr_char_value.p_value   = p_char_value;

    return sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, p_handles);
}

// Event Handle 
static void on_connect(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
    p_pbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
	p_pbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}
//uint32_t ble_pbs_dhrc_update(ble_pbs_t * p_pbs, uint8_t dhrc_data)
////uint32_t ble_drhc_update(ble_pbs_t * p_pbs, uint8_t* p_dhrc_data)
//{
//    if (p_pbs == NULL)
//    {
//        return NRF_ERROR_NULL;
//    }
//    
//    uint32_t err_code = NRF_SUCCESS;
//    ble_gatts_value_t gatts_value;
//		
//		// Initialize value struct.
//		memset(&gatts_value, 0, sizeof(gatts_value));

//		gatts_value.len     = 1;//DRHC_ALLDATA_LENGTH;
//		gatts_value.offset  = 0;
//		gatts_value.p_value = &dhrc_data;

//		// Update database.
//		err_code = sd_ble_gatts_value_set(p_pbs->conn_handle,
//																			p_pbs->drhc_handles.value_handle,
//																			&gatts_value);
//#if PBS_DEBUG
//		printf("set value err:%04X\r\n",err_code);
//#endif
//		// Send value if connected and notifying.
//		//if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
//		if (p_pbs->conn_handle != BLE_CONN_HANDLE_INVALID)
//		{
//				ble_gatts_hvx_params_t hvx_params;

//				memset(&hvx_params, 0, sizeof(hvx_params));

//				hvx_params.handle = p_pbs->drhc_handles.value_handle;
//				hvx_params.type   = BLE_GATT_HVX_INDICATION;
//				hvx_params.offset = gatts_value.offset;
//				hvx_params.p_len  = &gatts_value.len;
//				hvx_params.p_data = gatts_value.p_value;

//				err_code = sd_ble_gatts_hvx(p_pbs->conn_handle, &hvx_params);
//#if PBS_DEBUG
//			printf("indicate value err:%04X\r\n",err_code);
//#endif
//		}
//    return err_code;
//}

uint32_t ble_pbs_cdrc_update(ble_pbs_t * p_pbs, uint8_t *cdrc_data)
{
    if (p_pbs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
		
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = 20;
		gatts_value.offset  = 0;
		gatts_value.p_value = cdrc_data;

		// Update database.
		err_code = sd_ble_gatts_value_set(p_pbs->conn_handle,
																			p_pbs->cdrc_handles.value_handle,
																			&gatts_value);
#if PBS_DEBUG
		printf("set value err:%04X\r\n",err_code);
#endif
		// Send value if connected and notifying.
		//if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
		if (p_pbs->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				ble_gatts_hvx_params_t hvx_params;

				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_pbs->drhc_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_INDICATION;
				hvx_params.offset = gatts_value.offset;
				hvx_params.p_len  = &gatts_value.len;
				hvx_params.p_data = gatts_value.p_value;

				err_code = sd_ble_gatts_hvx(p_pbs->conn_handle, &hvx_params);
#if PBS_DEBUG
			printf("indicate value err:%04X\r\n",err_code);
#endif
		}
    return err_code;
}




static void on_write(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
		
	// Check if BSC's flag been writen
#if PBS_DEBUG
		printf("ble_pbs_on_ble_evt_on_write\r\n");
		uint16_t tempHandle = p_ble_evt->evt.gatts_evt.params.write.handle;
		printf("handle:%04X\r\n",tempHandle);
		uint8_t *tempData = p_ble_evt->evt.gatts_evt.params.write.data;
	//printf("data length:%d\r\n",sizeof(ble_gatts_evt_write_t)); //26
		printf("data[0][1]:%02X,%02X\r\n",tempData[0],tempData[1]);
#endif

uint32_t error_code;
// gill : Need drhc data encode afterward
uint8_t drhc_sim_data_ary_1[DRHC_ALLDATA_LENGTH] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
		
		if (tempHandle == 0x000D) // ESC value has been writting
		{
			p_pbs->esc_s.download_control_point = p_evt_write->data[0];
		}
		if(tempHandle == 0x000E) // ESC CCCD has been writting
    {
				//CCCD been written
        if ((p_evt_write->handle == p_pbs->esc_handles.cccd_handle) && (p_evt_write->len == 2))
				{
					esc_notify_flag = !esc_notify_flag;
#if PBS_DEBUG
					printf("esc_notify_flag:%d\r\n",esc_notify_flag);
#endif
				}
		}	
		if (tempHandle == 0x0011 && esc_notify_flag)
    {
        if ((p_evt_write->handle == p_pbs->drhc_handles.cccd_handle) && (p_evt_write->len == 2))
				{
					drhc_notify_flag = !drhc_notify_flag;
#if PBS_DEBUG
					printf("drhc_notify_flag:%d\r\n",drhc_notify_flag);
#endif
				}
		}	
		if (tempHandle == 0x0014 && esc_notify_flag && drhc_notify_flag)
    {
        if ((p_evt_write->handle == p_pbs->cdrc_handles.cccd_handle) && (p_evt_write->len == 2))
					cdrc_notify_flag = !cdrc_notify_flag;
		}	

}

void ble_pbs_on_ble_evt(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
    if (p_pbs == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_pbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_pbs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_pbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_pbs_init(ble_pbs_t * p_pbs)
{
		uint32_t   ser_err_code;
		uint32_t   char_err_code;
		ble_uuid_t service_uuid;
		ble_uuid_t char_uuid_temp;
		// Add service
		//BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_PIONEER_BEACON_SERVICE); // For SIG UUID

	// PBS Event Handle
		
	
	// Build GATT Profile 
		uint32_t err_code;
		ble_uuid128_t base_uuid = PBS_UUID_BASE;  // It's invert added from the array sequence, uint8_t [16] array
		err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type); // add to Nordic VS UUID table
		service_uuid.uuid = PBS_SERVICE_SHORT_UUID; // assign short UUID		
		ser_err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
#if PBS_DEBUG	  
		printf("vs_add:%d\r\n",err_code);	
		printf("service_add:%d\r\n",ser_err_code);	
#endif
		//Characteristic Setting
		// The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
		
		ble_gatts_attr_md_t cccd_md;
		ble_gatts_char_md_t char_md_temp;
		memset(&cccd_md, 0, sizeof(cccd_md));
		memset(&char_md_temp, 0, sizeof(char_md_temp));
		// Configure indicate
		
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		cccd_md.write_perm = p_pbs->pbs_cccd_md.cccd_write_perm;
		cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
		
		// Characteristic default setting 
    char_md_temp.char_props.read  = 0;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 0;
		char_md_temp.p_cccd_md        = NULL; //&cccd_md
    char_md_temp.p_char_user_desc = NULL;
    char_md_temp.p_char_pf        = NULL;
    char_md_temp.p_user_desc_md   = NULL;
    char_md_temp.p_sccd_md        = NULL;
	
		//p_pbs->bsc_s.flag = 0x01;  // Can not assign characteristic value here, show read-only variable can not assign
		// Beacon Setting Characteristic
		char_uuid_temp.uuid = PBS_BEACON_SETTING_CHAR_SHORT_UUID;
		uint8_t encoded_bsc_data [BSC_ALLDATA_LENGTH];
		bsc_data_encode(encoded_bsc_data, &p_pbs->bsc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_bsc_data,
														BSC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->bsc_handles);
#if PBS_DEBUG	
		printf("bsc_add:%d\r\n",char_err_code);
#endif											
												
		// Event Storage Characteristic
		char_uuid_temp.uuid = PBS_EVENT_STORAGE_CHAR_SHORT_UUID;
		uint8_t encoded_esc_data [ESC_ALLDATA_LENGTH];
		esc_data_encode(encoded_esc_data, &p_pbs->esc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_esc_data,
														ESC_ALLDATA_LENGTH,// Can not use sizeof(esc_t), or it will be 4
														&p_pbs->pbs_attr_md,
														&p_pbs->esc_handles);
		// Notify flag
		esc_notify_flag = false;
#if PBS_DEBUG			
		printf("esc_add:%d\r\n",char_err_code);
#endif														
		// Data Report Header Characteristic
		char_uuid_temp.uuid = PBS_DATA_REPORT_HEADER_CHAR_SHORT_UUID;
		uint8_t encoded_drhc_data [DRHC_ALLDATA_LENGTH];
		drhc_data_encode(encoded_drhc_data, &p_pbs->drhc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_drhc_data,
														DRHC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->drhc_handles);
		drhc_notify_flag = false;
#if PBS_DEBUG	
		printf("drhc_add:%d\r\n",char_err_code);												
#endif		
		// Cal Data Report Characteristic
		char_uuid_temp.uuid = PBS_CAL_DATA_REPORT_CHAR_SHORT_UUID;
		uint16_t CDRC_ALLDATA_LENGTH = p_pbs->cdrc_s.data_packet_length+2;
		uint8_t encoded_cdrc_data [CDRC_ALLDATA_LENGTH];
		cdrc_data_encode(encoded_cdrc_data, &p_pbs->cdrc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_cdrc_data,
														CDRC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->cdrc_handles);
		cdrc_notify_flag = false;
#if PBS_DEBUG	
		printf("cdrc_add:%d\r\n",char_err_code);
#endif		
		// Raw Data Report Characteristic
		char_uuid_temp.uuid = PBS_RAW_DATA_REPORT_CHAR_SHORT_UUID;
		uint16_t RDRC_ALLDATA_LENGTH = p_pbs->rdrc_s.data_packet_length+2;
		uint8_t encoded_rdrc_data [RDRC_ALLDATA_LENGTH];
		rdrc_data_encode(encoded_rdrc_data, &p_pbs->rdrc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_rdrc_data,
														RDRC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->rdrc_handles);
		rdrc_notify_flag = false;
#if PBS_DEBUG			
		printf("rdrc_add:%d\r\n",char_err_code);
#endif													

	return NRF_SUCCESS;
}

