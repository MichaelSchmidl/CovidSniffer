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
#define PIN_BCKL_ON      1               // GPIO value for backlight ON
#define PIN_BCKL_OFF     0               // GPIO value for backlight OFF

#define M5_PIN_NUM_AUDIO GPIO_NUM_25

#define DBG_PIN          GPIO_NUM_0

#define M5_PIN_NUM_BTN_A GPIO_NUM_39
#define M5_PIN_NUM_BTN_B GPIO_NUM_38
#define M5_PIN_NUM_BTN_C GPIO_NUM_37


#define PRIORITY_NORMAL       ( configMAX_PRIORITIES / 2 )
#define PRIORITY_LOWER        ( PRIORITY_NORMAL - 1 )
#define PRIORITY_HIGHER       ( PRIORITY_NORMAL + 1 )
#define PRIORITY_EVEN_LOWER   ( PRIORITY_LOWER - 1 )
#define PRIORITY_EVEN_HIGHER  ( PRIORITY_HIGHER + 1 )

#endif /*__HARDWARE_H__*/
