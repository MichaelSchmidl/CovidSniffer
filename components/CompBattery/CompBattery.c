/*
 * CompHDC2010.c
 *
 *  Created on: 05.12.2019
 *      Author: mschmidl
 */

#include <ctype.h>
#include "CompBattery.h"
#include "CompI2CDrv.h"


#define BATTERY_SA 0x75

static uint8_t _getBatteryU8FromAddr( const i2c_port_t i2cMasterPort, uint8_t addr )
{
	uint8_t val = i2cGetU8( i2cMasterPort,
			                BATTERY_SA,
					        addr );
	return val;
}

uint8_t getBatteryLevelPercent( const i2c_port_t i2cMasterPort )
{
	uint8_t val = _getBatteryU8FromAddr( i2cMasterPort, 0x78) & 0xF0;
	switch (val)
	{
        case 0xE0: return 25;
        case 0xC0: return 50;
        case 0x80: return 75;
        case 0x00: return 100;
        default:   return 0;
	}
    return 0;
}
