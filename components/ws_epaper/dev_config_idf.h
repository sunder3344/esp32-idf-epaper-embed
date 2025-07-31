#ifndef _DEV_CONFIG_IDF_H_
#define _DEV_CONFIG_IDF_H_

#include "driver/gpio.h"
#include <stdint.h>
#include "esp_err.h"

#define EPD_BUSY_PIN    20
#define EPD_RST_PIN     6
#define EPD_DC_PIN      18
#define EPD_CS_PIN      16
#define EPD_SCK_PIN     10
#define EPD_MOSI_PIN    11

esp_err_t DEV_Module_Init(void);
void DEV_GPIO_Init(void);
esp_err_t DEV_SPI_Init(void);
void DEV_SPI_WriteByte(uint8_t data);
void DEV_SPI_Write_nByte(uint8_t *data, uint16_t len);
void DEV_Digital_Write(gpio_num_t pin, uint8_t value);
uint8_t DEV_Digital_Read(gpio_num_t pin);
void DEV_Delay_ms(uint32_t xms);

#endif
