/*
 * CompCovidBeacon.h
 *
 *  Created on: 04.09.2020
 *      Author: mschmidl
 */

#ifndef COMP_COVID_BEACON_H_
#define COMP_COVID_BEACON_H_

#include <ctype.h>
#include "esp_bt_defs.h"

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/

#define LONG_EXPOSURE_SECONDS (3UL * 60UL)

typedef enum {
    unknown = 0,
    apple = 1,
    android = 2
} covidBeaconType_t;


uint16_t getNumberOfCovidBeacons( void );
uint16_t getNumberOfActiveCovidBeacons( void );
uint16_t getAgeOfCovidBeacon( uint32_t n );
uint16_t getNumberOfPossibleCovidBeacons( void );
uint16_t getMaxCovidBeacons( void );
uint32_t getTotalSumOfCovidBeacons( void );
uint32_t getMaxAgeOfCovidBeacons( void );
uint16_t getNumberOfLongExposedBeacons( void );
void clrAllCovidBeacons( void );
uint32_t getExposureTimeOfCovidBeacon( uint32_t n );

void gotNewAddr( esp_bd_addr_t newAddr, covidBeaconType_t beaconType );

#endif /* COMP_COVID_BEACON_H_ */
