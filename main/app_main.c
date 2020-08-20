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

#define MAX_COVID_ADDR_STORAGE 250
#define MAX_COVID_BEACON_AGE 90
typedef enum {
    unknown = 0,
    apple = 1,
    android = 2
} covidBeaconType_t;
typedef struct
{
	esp_bd_addr_t addr;
	covidBeaconType_t beaconType;
	uint16_t age;
	uint32_t first_seen;
	uint32_t last_seen;
} covidAppBeaconInfo_t;
static covidAppBeaconInfo_t covidAppBeaconInfo[MAX_COVID_ADDR_STORAGE];
static uint16_t maxTotalCovidBeacons = 0;
static uint32_t totalSumOfCovidBeacons = 0;

static void _decreaseAgeOfCovidBeacons( void )
{
	int n;
	for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
	{
//        esp_log_buffer_hex("old:", covidAppBeaconInfo[n].addr, sizeof(esp_bd_addr_t));
		if ( covidAppBeaconInfo[n].age > 0 )
		{
			covidAppBeaconInfo[n].age--;
		}
	}
}

uint16_t getNumberOfCovidBeacons( void )
{
    uint16_t cnt = 0;
	int n;
	for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
	{
		if ( covidAppBeaconInfo[n].age > 0 )
		{
			cnt++;
		}
	}

	return cnt;
}

uint16_t getNumberOfActiveCovidBeacons( void )
{
	_decreaseAgeOfCovidBeacons();
    uint16_t cnt = 0;
	int n;
	for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
	{
		if ( covidAppBeaconInfo[n].age > MAX_COVID_BEACON_AGE / 2 )
		{
			cnt++;
		}
	}

	return cnt;
}

uint16_t getNumberOfActiveAppleCovidBeacons( void )
{
    uint16_t cnt = 0;
    int n;
    for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
    {
        if ( covidAppBeaconInfo[n].age > MAX_COVID_BEACON_AGE / 2 )
        {
            if ( covidAppBeaconInfo[n].beaconType == apple )
            {
                cnt++;
            }
        }
    }

    return cnt;
}


uint16_t getNumberOfActiveAndroidCovidBeacons( void )
{
    uint16_t cnt = 0;
    int n;
    for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
    {
        if ( covidAppBeaconInfo[n].age > MAX_COVID_BEACON_AGE / 2 )
        {
            if ( covidAppBeaconInfo[n].beaconType == android )
            {
                cnt++;
            }
        }
    }

    return cnt;
}


uint32_t getExposureTimeOfCovidBeacon( uint32_t n )
{
    return covidAppBeaconInfo[n].last_seen - covidAppBeaconInfo[n].first_seen;
}

uint16_t getAgeOfCovidBeacon( uint32_t n )
{
	return covidAppBeaconInfo[n].age;
}

uint16_t getMaxCovidBeacons( void )
{
	return maxTotalCovidBeacons;
}

uint16_t getNumberOfPossibleCovidBeacons( void )
{
	return MAX_COVID_ADDR_STORAGE;
}

uint32_t getTotalSumOfCovidBeacons( void )
{
	return totalSumOfCovidBeacons;
}

uint32_t getMaxAgeOfCovidBeacons( void )
{
	return MAX_COVID_BEACON_AGE;
}

void clrAllCovidBeacons( void )
{
	int n;
	for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
	{
		memset( &covidAppBeaconInfo[n], 0 ,sizeof(covidAppBeaconInfo_t));
	}
	maxTotalCovidBeacons = 0;
	totalSumOfCovidBeacons = 0;
}

static void _gotNewAddr( esp_bd_addr_t newAddr, covidBeaconType_t beaconType )
{
	int n;
    TickType_t now = (xTaskGetTickCount() * portTICK_PERIOD_MS)  / 1000UL;
//    esp_log_buffer_hex("new:", newAddr, sizeof(esp_bd_addr_t));
	for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
	{
//        esp_log_buffer_hex("old:", covidAppBeaconInfo[n].addr, sizeof(esp_bd_addr_t));
		if ( 0 == memcmp( newAddr, covidAppBeaconInfo[n].addr, sizeof(esp_bd_addr_t) ) )
		{
		    // we already know about this addr
			if (covidAppBeaconInfo[n].age < MAX_COVID_BEACON_AGE )
			{
			    // if we did not see it for a while, set it back to MAX_AGE
				covidAppBeaconInfo[n].age = MAX_COVID_BEACON_AGE;
	            covidAppBeaconInfo[n].last_seen = now;
			}
			return;
		}
	}
	// if the addr is unknown so far, store it
	for ( n = 0; n < MAX_COVID_ADDR_STORAGE; n++ )
	{
		if ( covidAppBeaconInfo[n].age == 0 )
		{
			memcpy( covidAppBeaconInfo[n].addr, newAddr, sizeof(esp_bd_addr_t));
			covidAppBeaconInfo[n].age = MAX_COVID_BEACON_AGE;
			covidAppBeaconInfo[n].beaconType = beaconType;
		    covidAppBeaconInfo[n].first_seen = now;
            covidAppBeaconInfo[n].last_seen = now;
//			ESP_LOGI(__func__, "%d=%d", n, covidAppBeaconInfo[n].age);
			if ( n >= maxTotalCovidBeacons )
			{
				maxTotalCovidBeacons = n + 1;
			}
			totalSumOfCovidBeacons++;
			return;
		}
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
    covidBeaconType_t beaconType = unknown;

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
//                    I (40519) Beacon:: 00 00 00 00 3c e4 0a d2 7b 1b 00 00 02 00 00 00
//                    I (40519) Beacon:: 01 00 00 00 03 00 00 00 d5 ff ff ff 02 01 1a 03
//                    I (40519) Beacon:: 03 6f fd 17 16 6f fd fb 82 64 5e ee 14 8b 15 4f
//                    I (40519) Beacon:: d4 79 35 52 3b 5d f9 9e d4 ec dc 00 00 00 00 00
//                    I (40519) Beacon:: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
//                    I (40519) Beacon:: 00 00 00 00 00 00 00 00 00 00 00 00 1a 00 00 00
//                    I (40519) Beacon:: 01 00 00 00 1f 00 00 00 01 00 00 00
//                    I (40519)    ADV:: 02 01 1a 03 03 6f fd 17 16 6f fd fb 82 64 5e ee
//                    I (40519)    ADV:: 14 8b 15 4f d4 79 35 52 3b 5d f9 9e d4 ec dc
//                    ESP_LOGI(__func__, "---- Beacon ----");
//                    esp_log_buffer_hex("Beacon:", scan_result, sizeof(esp_ble_gap_cb_param_t));
//                    esp_log_buffer_hex("   ADV:", scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
                	if ( scan_result->scan_rst.adv_data_len >= 16 )
                	{
                		if (( scan_result->scan_rst.ble_adv[2] == 0x6F ) && ( scan_result->scan_rst.ble_adv[3] == 0xFD ))
                        {
                		    beaconType = android;
//                            ESP_LOGI(__func__, "---- androidCovidApp Beacon ----");
                        }
                        if (( scan_result->scan_rst.ble_adv[5] == 0x6F ) && ( scan_result->scan_rst.ble_adv[6] == 0xFD ))
						{
                            beaconType = apple;
//                            ESP_LOGI(__func__, "---- appleCovidApp Beacon ----");
						}
                        if ( beaconType != unknown )
                        {
//                            esp_log_buffer_hex("Addr:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
//                            esp_log_buffer_hex("Data: ", scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
                            _gotNewAddr( scan_result->scan_rst.bda, beaconType );
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
    clrAllCovidBeacons();
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
