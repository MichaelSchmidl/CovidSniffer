/*
 * CompCovidBeacon.c
 *
 *  Created on: 04.09.2020
 *      Author: mschmidl
 */

#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "CompCovidBeacon.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define MAX_COVID_ADDR_STORAGE 250
#define MAX_COVID_BEACON_AGE 90
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

uint16_t getNumberOfLongExposedBeacons( void )
{
    uint16_t cnt = 0;
    int n;
    for (n = 0; n < getNumberOfPossibleCovidBeacons(); n++ )
    {
        if ( getExposureTimeOfCovidBeacon(n) > LONG_EXPOSURE_SECONDS )
        {
            cnt++;
        }
    }
    return cnt;
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

void gotNewAddr( esp_bd_addr_t newAddr, covidBeaconType_t beaconType )
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
//          ESP_LOGI(__func__, "%d=%d", n, covidAppBeaconInfo[n].age);
            if ( n >= maxTotalCovidBeacons )
            {
                maxTotalCovidBeacons = n + 1;
            }
            totalSumOfCovidBeacons++;
            return;
        }
    }
}

