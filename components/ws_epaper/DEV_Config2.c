// dev_config_idf.c
// ESP-IDF 5.3 compatible version of microcontroller interface for Waveshare e-Paper

#include "dev_config_idf.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

spi_device_handle_t epd_spi;

void DEV_Delay_ms(uint32_t xms)
{
    vTaskDelay(pdMS_TO_TICKS(xms));
}

void DEV_GPIO_Init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << EPD_RST_PIN) |
                        (1ULL << EPD_DC_PIN) |
                        (1ULL << EPD_CS_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << EPD_BUSY_PIN);
    gpio_config(&io_conf);

    // Default levels
    gpio_set_level(EPD_CS_PIN, 1);
    //gpio_set_level(EPD_RST_PIN, 1);
    //gpio_set_level(EPD_DC_PIN, 1);
}

esp_err_t DEV_SPI_Init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = EPD_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = EPD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1, // We'll control CS manually
        .queue_size = 7,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &epd_spi));

    return ESP_OK;
}

void DEV_SPI_WriteByte(uint8_t data)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    gpio_set_level(EPD_CS_PIN, 0);
    spi_device_transmit(epd_spi, &t);
    gpio_set_level(EPD_CS_PIN, 1);
}

void DEV_SPI_Write_nByte(uint8_t *data, uint16_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    gpio_set_level(EPD_CS_PIN, 0);
    spi_device_transmit(epd_spi, &t);
    gpio_set_level(EPD_CS_PIN, 1);
}

uint8_t DEV_Digital_Read(gpio_num_t pin)
{
    return gpio_get_level(pin);
}

void DEV_Digital_Write(gpio_num_t pin, uint8_t value)
{
    gpio_set_level(pin, value);
}

esp_err_t DEV_Module_Init(void)
{
    DEV_GPIO_Init();
    return DEV_SPI_Init();
}
