#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include <driver/gpio.h>

#define M5_PIN_NUM_MISO  GPIO_NUM_19     // SPI MISO
#define M5_PIN_NUM_MOSI  GPIO_NUM_23     // SPI MOSI
#define M5_PIN_NUM_CLK   GPIO_NUM_18     // SPI CLOCK pin
#define M5_PIN_NUM_CS    GPIO_NUM_14     // Display CS pin
#define M5_PIN_NUM_DC    GPIO_NUM_27     // Display command/data pin
#define M5_PIN_NUM_TCS   GPIO_NUM_0      // Touch screen CS pin (NOT used if USE_TOUCH=0)

#define M5_PIN_NUM_RST   GPIO_NUM_33     // GPIO used for RESET control (#16)
#define M5_PIN_NUM_BCKL  GPIO_NUM_32     // GPIO used for backlight control
#define setBCKL_ON      1                // GPIO value for backlight ON
#define setBCKL_OFF     0                // GPIO value for backlight OFF

#define M5_PIN_NUM_AUDIO GPIO_NUM_25

#define DBG_PIN          GPIO_NUM_0

#define M5_PIN_NUM_BTN_A GPIO_NUM_39
#define M5_PIN_NUM_BTN_B GPIO_NUM_38
#define M5_PIN_NUM_BTN_C GPIO_NUM_37

#define M5_PIN_NUM_SCL   GPIO_NUM_22
#define M5_PIN_NUM_SDA   GPIO_NUM_21

#define PRIORITY_NORMAL       ( configMAX_PRIORITIES / 2 )
#define PRIORITY_LOWER        ( PRIORITY_NORMAL - 1 )
#define PRIORITY_HIGHER       ( PRIORITY_NORMAL + 1 )
#define PRIORITY_EVEN_LOWER   ( PRIORITY_LOWER - 1 )
#define PRIORITY_EVEN_HIGHER  ( PRIORITY_HIGHER + 1 )

#define I2C_PORT_NUM    I2C_NUM_1 // the I2C port number we will use to communicate with the connected chips

// PWM outputs by LEDC
#define BL_PWM_LEDC_CHANNEL            LEDC_CHANNEL_0

#define PWM_LEDC_MODE                  LEDC_HIGH_SPEED_MODE
#define PWM_LEDC_TIMER                 LEDC_TIMER_1
#define PWM_RESOLUTION                 LEDC_TIMER_12_BIT
#define PWM_FREQUENCY                  10000
#define PWM_REQUIRED_SIGNALS           1

#define BL_PWM                         0

#endif /*__HARDWARE_H__*/
