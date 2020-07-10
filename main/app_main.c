/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/****************************************************************************
*
* This file is used for eddystone receiver.
*
****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"

#include "hardware.h"
#include "ui_task.h"
#include "clockTask.h"
#include "ws2812.h"

#define MAX_COVID_ADDR_STORAGE 144
static int nCovidAppAddr = 0;
static esp_bd_addr_t covidAppAddr[MAX_COVID_ADDR_STORAGE];

int16_t getNumberOfCovidBeacons( void )
{
	return nCovidAppAddr;
}

void clrNumberOfCovidBeacons( void )
{
	nCovidAppAddr = 0;
	memset( covidAppAddr[nCovidAppAddr], 0 ,sizeof(esp_bd_addr_t));
}

static void addNewAddr( esp_bd_addr_t newAddr )
{
	int n;
//    esp_log_buffer_hex("new:", newAddr, sizeof(esp_bd_addr_t));
	for ( n = 0; n < nCovidAppAddr; n++ )
	{
//        esp_log_buffer_hex("old:", covidAppAddr[n], sizeof(esp_bd_addr_t));
		if ( 0 == memcmp( newAddr, covidAppAddr[n], sizeof(esp_bd_addr_t) ) )
		{
			return;
		}
	}
	if ( n < (MAX_COVID_ADDR_STORAGE - 1))
	{
		nCovidAppAddr++;
		memcpy( covidAppAddr[n], newAddr, sizeof(esp_bd_addr_t));
	}
}

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__func__,"Scan start failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(__func__,"Start scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                	if ( scan_result->scan_rst.adv_data_len >= 16 )
                	{
                		if (( scan_result->scan_rst.ble_adv[9] == 0x6F ) && ( scan_result->scan_rst.ble_adv[10] == 0xFD ))
						{
//                			ESP_LOGI(__func__, "---- CovidApp Beacon ----");
//                            esp_log_buffer_hex("Addr:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
//                            esp_log_buffer_hex("Data: ", scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
                            addNewAddr( scan_result->scan_rst.bda );
						}
                	}
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(__func__,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(__func__,"Stop scan successfully");
            }
            break;
        }
        default:
            break;
    }
}

void covid_sniffer_appRegister(void)
{
    esp_err_t status;
    
    ESP_LOGI(__func__,"Register callback");

    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(__func__,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void covid_sniffer_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    covid_sniffer_appRegister();
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    if(ws2812_init(15, LED_SK6812)) {
        ESP_LOGI(__func__, "ws2812_init failed: HALTING");
        while (true) {};
    }
    ws2812_allOff();

    UItask_init();
    UItask_connect();
    covid_sniffer_init();

    clockTaskStart( PRIORITY_NORMAL );

    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
}
