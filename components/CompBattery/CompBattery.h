/*
 * CompBattery.h
 *
 *  Created on: 12.07.2020
 *      Author: mschmidl
 */

#ifndef COMP_BATTERY_H_
#define COMP_BATTERY_H_

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
#include <driver/i2c.h>

uint8_t getBatteryLevelPercent( const i2c_port_t i2cMasterPort );

#endif /* COMP_BATTERY_H_ */
