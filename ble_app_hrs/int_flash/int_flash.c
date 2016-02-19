#include <int_flash.h>

#include "bsp.h"

#include <stdio.h>

void flash_page_erase(uint32_t * page_address)
{
    // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
void flash_word_write(uint32_t * address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

uint32_t flash_word_read(uint32_t * address)
{
    return *address;
}

/************ Following is used for Pioner Beacon Service ***********/
uint32_t start_addr = 0x60000;
uint32_t w_addr_ptr = 0x60000;
uint32_t r_addr_ptr = 0x60000;
uint32_t page_size = 0x1000;
uint32_t four_page_size = 4*0x1000;

#define BUFFER_EVENT_SIZE 		8
uint32_t w_r_addr[BUFFER_EVENT_SIZE];
uint32_t event_addr[BUFFER_EVENT_SIZE];
uint8_t event_type[BUFFER_EVENT_SIZE];
uint32_t event_utc[BUFFER_EVENT_SIZE];
uint8_t buffer_event_count = 0;
uint8_t w_buffer_event_ptr = 0;
uint8_t r_buffer_event_ptr = 0;
uint16_t count_down_7s_50Hz = 7*50;
uint16_t count_down_10s_50Hz = 10*50;
uint16_t to_addr_3s_50Hz = 3*50*16;
bool is_event_detected = false;

uint32_t w_flash_dummy1 = 0x0;
uint32_t w_flash_dummy2 = 0x0;
uint8_t bp;

int acc_x100_to_12bit[3];

void flash_data_set_init()
{		
		for (int i=0; i<BUFFER_EVENT_SIZE; i++)
			w_r_addr[i] = start_addr + i*four_page_size; 
}

uint8_t flash_data_set_write(float acc_g_xyz[3], float cal_acc_g_xyz[3], uint8_t event_ID, uint32_t UTC)
{
		uint32_t temp;
		if (buffer_event_count==BUFFER_EVENT_SIZE) return buffer_event_count;// buffer full return immediately  
			
		if (event_ID) {
			is_event_detected = true;
			event_addr[w_buffer_event_ptr] = w_addr_ptr;
			event_type[w_buffer_event_ptr] = event_ID;
			event_utc[w_buffer_event_ptr] = UTC;
		}
		
		if (w_addr_ptr%page_size == 0) flash_page_erase((uint32_t*)w_addr_ptr);// new page, erase before writing
	
		if (acc_g_xyz != NULL) {
			flash_word_write((uint32_t*)w_addr_ptr, w_flash_dummy1++);
		}
		w_addr_ptr+=4;

		if (acc_g_xyz != NULL) {		
			flash_word_write((uint32_t*)w_addr_ptr, w_flash_dummy1++);
		}
		w_addr_ptr+=4;

		if (cal_acc_g_xyz != NULL) {
			temp = (uint32_t) ((float)(cal_acc_g_xyz[0]+16)*100)<<16 | (uint32_t) ((float)(cal_acc_g_xyz[1]+16)*100); 
			flash_word_write((uint32_t*)w_addr_ptr, temp);
		}	
		w_addr_ptr+=4;

		if (cal_acc_g_xyz != NULL) {		
			temp = (uint32_t) ((float)(cal_acc_g_xyz[2]+16)*100); 
			flash_word_write((uint32_t*)w_addr_ptr, temp);
		}
		w_addr_ptr+=4;
	
		if (w_addr_ptr == w_r_addr[w_buffer_event_ptr]+four_page_size) // ring buffering management, round in 4 pase size
			w_addr_ptr = w_r_addr[w_buffer_event_ptr];
//		if (w_addr_ptr%four_page_size == 0)
//			bp++;
		
		if (is_event_detected) 
			count_down_7s_50Hz--;
		if (count_down_7s_50Hz==0) {
			is_event_detected = false;
			count_down_7s_50Hz = 7*50;
			w_buffer_event_ptr++;
			if(w_buffer_event_ptr==BUFFER_EVENT_SIZE) w_buffer_event_ptr = 0;
			w_addr_ptr = w_r_addr[w_buffer_event_ptr];
			buffer_event_count++;
		}
		return buffer_event_count;
}	

int flag = 1;

int flash_data_set_read(uint32_t *acc_g_xy, uint32_t *acc_g_z, uint32_t *cal_acc_g_xy, uint32_t *cal_acc_g_z, uint8_t *event_ID, uint32_t *UTC)
{
		if (buffer_event_count == 0) return -1;// buffer empty return immediately  
		
		if (flag) {
			count_down_10s_50Hz=10*50;
			r_addr_ptr = event_addr[r_buffer_event_ptr]-to_addr_3s_50Hz;
			if (r_addr_ptr < w_r_addr[r_buffer_event_ptr]) r_addr_ptr+= four_page_size;
			*event_ID = event_type[r_buffer_event_ptr];
			*UTC = event_utc[r_buffer_event_ptr];
		}
		flag = 0;
		
		if (acc_g_xy != NULL) {
			*acc_g_xy = flash_word_read((uint32_t *)r_addr_ptr);
		}
		r_addr_ptr+=4;
		
		if (acc_g_z != NULL) {
			*acc_g_z = flash_word_read((uint32_t *)r_addr_ptr);
		}
		r_addr_ptr+=4;
		
		if (cal_acc_g_xy != NULL) {
			*cal_acc_g_xy = flash_word_read((uint32_t *)r_addr_ptr);
		}
		r_addr_ptr+=4;
		
		if (cal_acc_g_z != NULL) {
			*cal_acc_g_z = flash_word_read((uint32_t *)r_addr_ptr);	
		}
		r_addr_ptr+=4;

		if (r_addr_ptr == w_r_addr[r_buffer_event_ptr]+four_page_size) // ring buffering management, round in 4 pase size
			w_addr_ptr = w_r_addr[r_buffer_event_ptr];
		
		count_down_10s_50Hz--;
		if (count_down_10s_50Hz==0) {
			r_buffer_event_ptr++;
			if(r_buffer_event_ptr==BUFFER_EVENT_SIZE) r_buffer_event_ptr = 0;
			buffer_event_count--;
			flag = 1;
		}
		return count_down_10s_50Hz;
}	