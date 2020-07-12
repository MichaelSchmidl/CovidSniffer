/*
 * CompI2CDrv.c
 *
 *  Created on: 04.12.2019
 *      Author: mschmidl
 */

#include "CompI2CDrv.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2cWrite(const i2c_port_t i2c_num, const uint8_t slaveAddr7, const uint8_t *data_wr, const size_t size, const uint8_t withStop)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte( cmd,
			               (slaveAddr7 << 1) | I2C_MASTER_WRITE,
						   ACK_CHECK_EN ); // check ACK
	i2c_master_write( cmd,
			          (uint8_t*)data_wr,
					  size,
					  ACK_CHECK_EN ); // check ACK
	if ( 0 != withStop )
	{
		i2c_master_stop(cmd);
	}
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}


/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2cRead(const i2c_port_t i2c_num, const uint8_t slaveAddr7, uint8_t *data_rd, const size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr7 << 1) | I2C_MASTER_READ, 1);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


uint16_t i2cGetU16( const i2c_port_t i2cMasterPort, const uint8_t slaveAddr7, const uint8_t regAddr )
{
	uint16_t ret = 0;
	if (ESP_OK == i2cWrite( i2cMasterPort,
			                slaveAddr7,
			                &regAddr,
			                sizeof(regAddr),
							0 ) )
	{
		if ( ESP_OK != i2cRead( i2cMasterPort,
				                slaveAddr7,
								(uint8_t*)&ret,
								sizeof(ret) ))
		{
			ret = 0;
		}
	}
	return ret;
}


uint8_t i2cGetU8( const i2c_port_t i2cMasterPort, const uint8_t slaveAddr7, const uint8_t regAddr )
{
	uint8_t ret = 0;
	if (ESP_OK == i2cWrite( i2cMasterPort,
			                slaveAddr7,
			                &regAddr,
			                sizeof(regAddr),
							0 ) )
	{
		if ( ESP_OK != i2cRead( i2cMasterPort,
				                slaveAddr7,
								(uint8_t*)&ret,
								sizeof(ret) ))
		{
			ret = 0;
		}
	}
	return ret;
}


esp_err_t i2cInit( const gpio_num_t sclPin,
		      const gpio_num_t sdaPin,
			  const uint32_t speedHz,
			  const i2c_port_t i2cMasterPort )
{
	printf( "%s\n", __func__ );

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdaPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = sclPin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = speedHz;
    i2c_param_config( i2cMasterPort, &conf );
    return i2c_driver_install( i2cMasterPort,
    		                   conf.mode,
                               0,   // no TX_BUF necessary
                               0,   // no RX_BUF necessary
							   0 ); // no interrupt
}
