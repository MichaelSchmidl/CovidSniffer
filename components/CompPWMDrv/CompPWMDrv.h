/*
 * CompPWMDrv.h
 *
 *  Created on: 24.07.2020
 *      Author: mschmidl
 */

#ifndef COMP_PWM_DRV_H_
#define COMP_PWM_DRV_H_

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

esp_err_t initPWMs( void );
esp_err_t pwmSetDutyCycle(int whichPWM, uint16_t val);

#endif /* COMP_PWM_DRV_H_ */
