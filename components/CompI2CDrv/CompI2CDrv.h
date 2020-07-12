/*
 * CompI2CDrv.h
 *
 *  Created on: 04.12.2019
 *      Author: mschmidl
 */

#ifndef COMP_I2C_DRV_H_
#define COMP_I2C_DRV_H_

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

uint16_t i2cGetU16( const i2c_port_t i2cMasterPort,
		            const uint8_t slaveAddr7,
					const uint8_t regAddr );

uint8_t i2cGetU8( const i2c_port_t i2cMasterPort,
		          const uint8_t slaveAddr7,
				  const uint8_t regAddr );

esp_err_t i2cWrite( const i2c_port_t i2c_num,
		            const uint8_t slaveAddr7,
					const uint8_t *data_wr,
					const size_t size,
					const uint8_t withStop );

esp_err_t i2cRead( const i2c_port_t i2c_num,
		           const uint8_t slaveAddr7,
				   uint8_t *data_rd,
				   const size_t size );

esp_err_t i2cInit( const gpio_num_t sclPin,
		      const gpio_num_t sdaPin,
			  const uint32_t speedHz,
			  const i2c_port_t i2cMasterPort );

#endif /* COMP_I2C_DRV_H_ */
