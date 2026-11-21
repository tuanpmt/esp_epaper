/**
 * @file gdey0154_lut.c
 * @brief Driver for GDEY0154D67 1.54" B/W e-paper display with custom LUT
 *
 * This panel uses custom waveform lookup tables for optimized
 * full and partial refresh performance.
 *
 * 4-gray mode (EPD_PANEL_GDEY0154D67_4GRAY): uses the 4-gray waveform stored in
 * the panel OTP, selected by forcing the temperature register to 0x5A, as in the
 * Good Display reference code (EPD_HW_Init_4G / EPD_WhiteScreen_ALL_4G).
 * The 2-bit framebuffer is split into two bit planes written to RAM 0x24/0x26.
 */

#include "../epaper_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>

static const char *TAG = "gdey0154";

/*=============================================================================
 * Waveform LUTs
 *============================================================================*/

static const uint8_t LUT_FULL[159] = {
    0x80, 0x48, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x40, 0x48, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x80, 0x48, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x40, 0x48, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0xA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x8, 0x1, 0x0, 0x8, 0x1, 0x0, 0x2,
    0xA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
    0x22, 0x17, 0x41, 0x0, 0x32, 0x20
};

static const uint8_t LUT_PARTIAL[159] = {
    0x0, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x40, 0x40, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0xF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x0, 0x0, 0x0,
    0x02, 0x17, 0x41, 0xB0, 0x32, 0x28,
};

/*=============================================================================
 * LUT Loading
 *============================================================================*/

static void load_lut(epd_spi_t *spi, const uint8_t *lut)
{
    CMD(spi, 0x32);
    epd_spi_write_data_bulk(spi, lut, 153);
    epd_spi_wait_busy(spi, 0);

    CMD(spi, 0x3F);
    DATA(spi, lut[153]);

    CMD(spi, 0x03);
    DATA(spi, lut[154]);

    CMD(spi, 0x04);
    DATA(spi, lut[155]);
    DATA(spi, lut[156]);
    DATA(spi, lut[157]);

    CMD(spi, 0x2C);
    DATA(spi, lut[158]);
}

static inline bool is_4gray(epd_device_t *dev)
{
    return epd_get_panel(dev)->color_mode == EPD_COLOR_4GRAY;
}

// Select the OTP 4-gray waveform (temperature register forced to 0x5A)
static void load_4gray_waveform(epd_spi_t *spi)
{
    CMD(spi, 0x1A);  // Write temperature register
    DATA(spi, 0x5A);
    DATA(spi, 0x00);

    CMD(spi, 0x22);  // Load temperature value + LUT from OTP
    DATA(spi, 0x91);
    CMD(spi, 0x20);
    epd_spi_wait_busy(spi, 0);
}

/*=============================================================================
 * Initialization
 *============================================================================*/

esp_err_t gdey0154_init(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);
    uint16_t w = epd_get_width(dev);
    uint16_t h = epd_get_height(dev);

    RESET(spi);
    vTaskDelay(pdMS_TO_TICKS(50));

    epd_spi_wait_busy(spi, 0);
    CMD(spi, 0x12);  // SWRESET
    epd_spi_wait_busy(spi, 0);

    CMD(spi, 0x01);  // Driver output control
    DATA(spi, 0xC7);
    DATA(spi, 0x00);
    DATA(spi, 0x01);

    CMD(spi, 0x11);  // Data entry mode
    DATA(spi, 0x01);

    // Set RAM window
    CMD(spi, 0x44);
    DATA(spi, 0);
    DATA(spi, (w - 1) / 8);

    CMD(spi, 0x45);
    DATA(spi, (h - 1) & 0xFF);
    DATA(spi, ((h - 1) >> 8) & 0xFF);
    DATA(spi, 0);
    DATA(spi, 0);

    CMD(spi, 0x3C);  // Border waveform
    DATA(spi, 0x01);

    CMD(spi, 0x18);  // Temperature sensor
    DATA(spi, 0x80);

    CMD(spi, 0x22);  // Load temperature
    DATA(spi, 0xB1);
    CMD(spi, 0x20);

    // Set cursor
    CMD(spi, 0x4E);
    DATA(spi, 0);

    CMD(spi, 0x4F);
    DATA(spi, (h - 1) & 0xFF);
    DATA(spi, ((h - 1) >> 8) & 0xFF);

    epd_spi_wait_busy(spi, 0);
    if (is_4gray(dev)) {
        load_4gray_waveform(spi);
    } else {
        load_lut(spi, LUT_FULL);
    }

    ESP_LOGD(TAG, "Init complete (%dx%d%s)", w, h, is_4gray(dev) ? ", 4-gray" : "");
    return ESP_OK;
}

/*=============================================================================
 * Update Functions
 *============================================================================*/

static esp_err_t gdey0154_update_full(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    load_lut(spi, LUT_FULL);

    CMD(spi, 0x22);
    DATA(spi, 0xC7);
    CMD(spi, 0x20);
    epd_spi_wait_busy(spi, 0);

    return ESP_OK;
}

static esp_err_t gdey0154_update_partial(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    RESET(spi);
    vTaskDelay(pdMS_TO_TICKS(50));
    epd_spi_wait_busy(spi, 0);

    load_lut(spi, LUT_PARTIAL);

    CMD(spi, 0x37);
    DATA(spi, 0x00);
    DATA(spi, 0x00);
    DATA(spi, 0x00);
    DATA(spi, 0x00);
    DATA(spi, 0x00);
    DATA(spi, 0x40);
    DATA(spi, 0x00);
    DATA(spi, 0x00);
    DATA(spi, 0x00);
    DATA(spi, 0x00);

    CMD(spi, 0x3C);  // Border waveform
    DATA(spi, 0x80);

    CMD(spi, 0x22);
    DATA(spi, 0xCF);
    CMD(spi, 0x20);
    epd_spi_wait_busy(spi, 0);

    return ESP_OK;
}

static esp_err_t gdey0154_update_4gray(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    CMD(spi, 0x22);  // Display update with the loaded 4-gray waveform
    DATA(spi, 0xC7);
    CMD(spi, 0x20);
    epd_spi_wait_busy(spi, 0);

    return ESP_OK;
}

esp_err_t gdey0154_update(epd_device_t *dev, epd_update_mode_t mode)
{
    if (is_4gray(dev)) {
        return gdey0154_update_4gray(dev);
    }

    switch (mode) {
        case EPD_UPDATE_PARTIAL:
        case EPD_UPDATE_FAST:
            return gdey0154_update_partial(dev);
        case EPD_UPDATE_FULL:
        default:
            return gdey0154_update_full(dev);
    }
}

/*=============================================================================
 * RAM Write Functions
 *============================================================================*/

// Reset RAM window and address counter to the full panel
static void set_full_window(epd_spi_t *spi, uint16_t w, uint16_t h)
{
    CMD(spi, 0x44);
    DATA(spi, 0);
    DATA(spi, (w - 1) / 8);

    CMD(spi, 0x45);
    DATA(spi, (h - 1) & 0xFF);
    DATA(spi, ((h - 1) >> 8) & 0xFF);
    DATA(spi, 0);
    DATA(spi, 0);

    CMD(spi, 0x4E);
    DATA(spi, 0);
    CMD(spi, 0x4F);
    DATA(spi, (h - 1) & 0xFF);
    DATA(spi, ((h - 1) >> 8) & 0xFF);
}

/*
 * Write a 2-bit framebuffer (4 pixels per byte, MSB first, 3 = white) as two
 * bit planes. Mapping from the Good Display reference (after its inversion):
 *   RAM 0x24 bit = !(level bit 0), RAM 0x26 bit = !(level bit 1)
 *   white (3) -> 0/0, light (2) -> 1/0, dark (1) -> 0/1, black (0) -> 1/1
 */
static esp_err_t gdey0154_write_ram_4gray(epd_device_t *dev, const uint8_t *data, uint32_t len)
{
    epd_spi_t *spi = epd_get_spi(dev);
    uint16_t w = epd_get_width(dev);
    uint16_t h = epd_get_height(dev);
    uint32_t plane_size = ((uint32_t)w * h) / 8;

    if (len < plane_size * 2) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *plane = heap_caps_malloc(plane_size, MALLOC_CAP_DMA);
    if (!plane) {
        return ESP_ERR_NO_MEM;
    }

    static const uint8_t ram_cmd[2] = { 0x24, 0x26 };
    for (int p = 0; p < 2; p++) {
        // p = 0: level bit 0 -> RAM 0x24, p = 1: level bit 1 -> RAM 0x26
        for (uint32_t i = 0; i < plane_size; i++) {
            uint8_t hi = data[i * 2];
            uint8_t lo = data[i * 2 + 1];
            uint8_t out = 0;
            for (int k = 0; k < 4; k++) {
                out = (out << 1) | ((hi >> (6 - k * 2 + p)) & 1);
            }
            for (int k = 0; k < 4; k++) {
                out = (out << 1) | ((lo >> (6 - k * 2 + p)) & 1);
            }
            plane[i] = ~out;
        }
        set_full_window(spi, w, h);
        CMD(spi, ram_cmd[p]);
        epd_spi_write_data_bulk(spi, plane, plane_size);
    }

    free(plane);
    return ESP_OK;
}

esp_err_t gdey0154_write_ram(epd_device_t *dev, const uint8_t *data, uint32_t len)
{
    if (is_4gray(dev)) {
        return gdey0154_write_ram_4gray(dev, data, len);
    }

    epd_spi_t *spi = epd_get_spi(dev);
    bool partial_ready = epd_is_partial_ready(dev);

    // Write to current RAM (0x24)
    CMD(spi, 0x24);
    epd_spi_write_data_bulk(spi, data, len);

    // Only write to base image RAM (0x26) when setting up for partial refresh
    // When partial_ready is true, we only update 0x24 so the display can
    // compare old (0x26) vs new (0x24) for partial update
    if (!partial_ready) {
        CMD(spi, 0x26);
        epd_spi_write_data_bulk(spi, data, len);
    }

    return ESP_OK;
}

esp_err_t gdey0154_write_ram_partial(epd_device_t *dev, uint16_t x, uint16_t y,
                                      uint16_t w, uint16_t h, const uint8_t *data)
{
    if (is_4gray(dev)) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    epd_spi_t *spi = epd_get_spi(dev);
    uint16_t panel_h = epd_get_height(dev);

    uint16_t x_start = x / 8;
    uint16_t x_end = x_start + w / 8 - 1;
    uint16_t y_start = panel_h - 1 - y;
    uint16_t y_end = y_start - h + 1;

    CMD(spi, 0x44);
    DATA(spi, x_start);
    DATA(spi, x_end);

    CMD(spi, 0x45);
    DATA(spi, y_start & 0xFF);
    DATA(spi, (y_start >> 8) & 0xFF);
    DATA(spi, y_end & 0xFF);
    DATA(spi, (y_end >> 8) & 0xFF);

    CMD(spi, 0x4E);
    DATA(spi, x_start);

    CMD(spi, 0x4F);
    DATA(spi, y_start & 0xFF);
    DATA(spi, (y_start >> 8) & 0xFF);

    CMD(spi, 0x24);
    epd_spi_write_data_bulk(spi, data, (w / 8) * h);

    return ESP_OK;
}

/*=============================================================================
 * Power Management
 *============================================================================*/

esp_err_t gdey0154_sleep(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    CMD(spi, 0x10);
    DATA(spi, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Panel in deep sleep");
    return ESP_OK;
}

esp_err_t gdey0154_wake(epd_device_t *dev)
{
    return gdey0154_init(dev);
}
