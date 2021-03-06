/*
 * A driver for the WS2812 RGB LEDs using the RMT peripheral on the ESP32.
 *
 * Modifications Copyright (c) 2017 Martin F. Falatic
 *
 * Based on public domain code created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 */
/* 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdint.h>

#define NUM_PIXELS 10  // How many pixels you want to drive

#define RGB_VAL(r, g, b) (uint32_t)((r << 0) | (g << 8) | (b << 16))

typedef union {
  struct __attribute__ ((packed)) {
    uint8_t r, g, b;
  };
  uint32_t num;
} rgbVal;

inline rgbVal makeRGBVal(uint8_t r, uint8_t g, uint8_t b)
{
  rgbVal v;
  v.r = r;
  v.g = g;
  v.b = b;
  return v;
}

enum led_types {LED_WS2812, LED_WS2812B, LED_SK6812, LED_WS2813};
int  ws2812_init(int gpioNum, int ledType);

void ws2812_setColors(uint16_t length, rgbVal *array);

void ws2812_allOff(void);

void ws2812_setLED(int n, uint32_t color);

void ws2812_refreshLEDs( void );

#endif /* WS2812_DRIVER_H */
