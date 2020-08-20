/*
 * CompPWMDrv.c
 *
 *  Created on: 24.07.2020
 *      Author: mschmidl
 */

#include <esp_log.h>
#include "hardware.h"
#include "CompPWMDrv.h"

static ledc_channel_config_t ledc_channel[PWM_REQUIRED_SIGNALS] = {
    {
        //set LEDC channel
        .channel = BL_PWM_LEDC_CHANNEL,
        //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
        .duty = 0, // 0% by default
        //GPIO number
        .gpio_num = M5_PIN_NUM_BCKL,
        //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
        .intr_type = LEDC_INTR_DISABLE,
        //set LEDC mode, from ledc_mode_t
        .speed_mode = PWM_LEDC_MODE,
        //set LEDC timer source, if different channel use one timer,
        //the frequency and bit_num of these channels should be the same
        .timer_sel = PWM_LEDC_TIMER
    }
};


esp_err_t pwmSetDutyCycle(int whichPWM, uint16_t val)
{
    if (whichPWM >= PWM_REQUIRED_SIGNALS) return ESP_ERR_NOT_SUPPORTED;
    if ( val >= (1 << PWM_RESOLUTION) ) return ESP_ERR_NOT_SUPPORTED;

    esp_err_t ret = ledc_set_duty(ledc_channel[whichPWM].speed_mode, ledc_channel[whichPWM].channel, val );
    if ( ESP_OK == ret)
    {
       ret = ledc_update_duty(ledc_channel[whichPWM].speed_mode, ledc_channel[whichPWM].channel);
    }
    return ret;
}


esp_err_t initPWMs( void )
{
    printf( "%s\n", __func__ );
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,          //set timer counter bit number
        .freq_hz = PWM_FREQUENCY,                   //set frequency of pwm
        .speed_mode = PWM_LEDC_MODE,                //timer mode,
        .timer_num = PWM_LEDC_TIMER                 //timer index
    };
    //configure timer0 for high speed channels
    if ( ESP_OK != ledc_timer_config(&ledc_timer))
    {
        ESP_LOGE(__func__, "ledc_timer_config failed");
        return ESP_FAIL;
    }

    //set the configuration
    for (int ch = 0; ch < PWM_REQUIRED_SIGNALS; ch++)
    {
       if (ESP_OK != ledc_channel_config(&ledc_channel[ch]))
       {
           ESP_LOGE(__func__, "ledc_channel_config failed");
           return ESP_FAIL;
       }
    }

    return ESP_OK;
}
