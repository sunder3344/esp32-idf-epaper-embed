/*****************************************************************************
* | File      	:   DEV_Config.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2020-02-19
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdio.h>

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO config
**/
#define EPD_SCK_PIN  10
#define EPD_MOSI_PIN 11
#define EPD_CS_PIN   16
#define EPD_RST_PIN  6
#define EPD_DC_PIN   18
#define EPD_BUSY_PIN 20

#define GPIO_PIN_SET   1
#define GPIO_PIN_RESET 0

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) gpio_set_level(_pin, (_value == 0) ? 0 : 1)
#define DEV_Digital_Read(_pin) gpio_get_level(_pin)

/**
 * delay x ms
**/
#define DEV_Delay_ms(__xms) vTaskDelay(pdMS_TO_TICKS(__xms / portTICK_PERIOD_MS))

/*------------------------------------------------------------------------------------------------------*/
UBYTE DEV_Module_Init(void);
void GPIO_Mode(UWORD GPIO_Pin, UWORD Mode);
void DEV_SPI_WriteByte(UBYTE data);
UBYTE DEV_SPI_ReadByte();
void DEV_SPI_Write_nByte(UBYTE *pData, UDOUBLE len);

#endif
