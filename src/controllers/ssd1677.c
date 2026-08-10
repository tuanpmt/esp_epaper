/**
 * @file ssd1677.c
 * @brief Driver for SSD1677 based B/W e-paper displays (GDEQ0426T82)
 *
 * Supports the 4.26" 800x480 panel used by the Xteink X4. Unlike the SSD1680/1681
 * family, SSD1677 addresses RAM with pixel-based X ranges, decrements Y with a
 * physically reversed gate scan, and runs its refresh sequences from OTP (0x22
 * values 0xF7 full / 0xFC fast-partial). Command sequences are lifted from the
 * FreeInk and CrossPoint community-sdk drivers, both proven on the X4 panel.
 */

#include "../epaper_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ssd1677";

/*=============================================================================
 * RAM Window
 *============================================================================*/

static void ssd1677_set_ram_area(epd_device_t *dev, uint16_t x, uint16_t y,
                                  uint16_t w, uint16_t h)
{
    epd_spi_t *spi = epd_get_spi(dev);
    uint16_t panel_h = epd_get_height(dev);

    // Gates are physically reversed on this panel: reverse the window so the
    // byte stream fills rows top-to-bottom.
    y = panel_h - y - h;

    // Data entry: X increment, Y decrement.
    CMD(spi, 0x11);  // Set RAM entry mode
    DATA(spi, 0x01);

    // X range/counter addresses are in pixels, 2 bytes each (10-bit).
    CMD(spi, 0x44);  // Set RAM X address range
    DATA(spi, x & 0xFF);
    DATA(spi, x >> 8);
    DATA(spi, (x + w - 1) & 0xFF);
    DATA(spi, (x + w - 1) >> 8);

    CMD(spi, 0x45);  // Set RAM Y address range
    DATA(spi, (y + h - 1) & 0xFF);
    DATA(spi, (y + h - 1) >> 8);
    DATA(spi, y & 0xFF);
    DATA(spi, y >> 8);

    CMD(spi, 0x4E);  // Set RAM X address counter
    DATA(spi, x & 0xFF);
    DATA(spi, x >> 8);

    CMD(spi, 0x4F);  // Set RAM Y address counter
    DATA(spi, (y + h - 1) & 0xFF);
    DATA(spi, (y + h - 1) >> 8);
}

/*=============================================================================
 * Initialization
 *============================================================================*/

esp_err_t ssd1677_init(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);
    uint16_t w = epd_get_width(dev);
    uint16_t h = epd_get_height(dev);

    ESP_LOGI(TAG, "Init SSD1677 panel: %dx%d", w, h);

    // Wait for any in-flight refresh to finish before the reset pin toggles,
    // otherwise the reset aborts the drive mid-frame and leaves a partial image.
    WAIT(dev);

    RESET(spi);
    vTaskDelay(pdMS_TO_TICKS(10));
    WAIT(dev);

    CMD(spi, 0x12);  // SWRESET
    WAIT(dev);

    CMD(spi, 0x18);  // Temperature sensor control
    DATA(spi, 0x80);  // Internal sensor

    CMD(spi, 0x0C);  // Booster soft-start (Good-Display/FreeInk X4 values)
    DATA(spi, 0xAE);
    DATA(spi, 0xC7);
    DATA(spi, 0xC3);
    DATA(spi, 0xC0);
    DATA(spi, 0x80);

    CMD(spi, 0x01);  // Driver output control
    DATA(spi, (h - 1) & 0xFF);
    DATA(spi, (h - 1) >> 8);
    DATA(spi, 0x02);  // SM=1 (interlaced), TB=0

    CMD(spi, 0x3C);  // Border waveform
    DATA(spi, 0x80);  // X4 init border

    ssd1677_set_ram_area(dev, 0, 0, w, h);

    // Clear both RAM planes to white. The auto-write commands fill RAM on the
    // controller without host data transfers.
    CMD(spi, 0x46);  // Auto write BW RAM
    DATA(spi, 0xFF);
    WAIT(dev);

    CMD(spi, 0x47);  // Auto write RED RAM
    DATA(spi, 0xFF);
    WAIT(dev);

    ESP_LOGI(TAG, "Panel initialized");
    return ESP_OK;
}

/*=============================================================================
 * Update Functions
 *============================================================================*/

static esp_err_t ssd1677_update_full(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    // Display update control: bypass RED, single chip application.
    CMD(spi, 0x21);
    DATA(spi, 0x40);
    DATA(spi, 0x00);

    CMD(spi, 0x3C);  // Border waveform for full refresh
    DATA(spi, 0xC0);

    CMD(spi, 0x22);  // Display update (loads LUT, drives, powers down)
    DATA(spi, 0xF7);

    CMD(spi, 0x20);  // Master activation
    WAIT(dev);

    return ESP_OK;
}

static esp_err_t ssd1677_update_fast(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    // Display update control: RED normal (differential vs the base image),
    // single chip application.
    CMD(spi, 0x21);
    DATA(spi, 0x00);
    DATA(spi, 0x00);

    CMD(spi, 0x3C);  // Border waveform for fast refresh
    DATA(spi, 0xC0);

    CMD(spi, 0x22);  // Display update (partial/fast sequence, keeps panel powered)
    DATA(spi, 0xFC);

    CMD(spi, 0x20);  // Master activation
    WAIT(dev);

    return ESP_OK;
}

esp_err_t ssd1677_update(epd_device_t *dev, epd_update_mode_t mode)
{
    switch (mode) {
        case EPD_UPDATE_FAST:
        case EPD_UPDATE_PARTIAL:
            // The panel has no distinct partial sequence; fast is the same
            // differential 0xFC refresh the stock X4 firmware uses for partial.
            return ssd1677_update_fast(dev);
        case EPD_UPDATE_FULL:
        default:
            return ssd1677_update_full(dev);
    }
}

/*=============================================================================
 * RAM Write Functions
 *============================================================================*/

esp_err_t ssd1677_write_ram(epd_device_t *dev, const uint8_t *data, uint32_t len)
{
    epd_spi_t *spi = epd_get_spi(dev);
    uint16_t w = epd_get_width(dev);
    uint16_t h = epd_get_height(dev);
    bool partial_ready = epd_is_partial_ready(dev);

    // GDDRAM is read-only while the panel is refreshing: writes issued during
    // BUSY are ignored. Wait for idle before the stream. This is the async
    // path's synchronization point - it always runs, unlike the post-trigger
    // busy wait in the update sequences, which epd_update_async() defers.
    WAIT(dev);

    ssd1677_set_ram_area(dev, 0, 0, w, h);

    // Write to current RAM (0x24)
    CMD(spi, 0x24);
    epd_spi_write_data_bulk(spi, data, len);

    // Only write to base image RAM (0x26) when setting up for partial refresh.
    // While partial_ready, 0x26 holds the frame the controller diffs against,
    // so only 0x24 is updated.
    if (!partial_ready) {
        ssd1677_set_ram_area(dev, 0, 0, w, h);

        CMD(spi, 0x26);
        epd_spi_write_data_bulk(spi, data, len);
    }

    return ESP_OK;
}

// Windowed variant of ssd1677_write_ram(): streams one byte-aligned rectangle
// into the display RAM (0x24) only. The caller is responsible for keeping the
// base image RAM (0x26) in sync via ssd1677_write_ram_partial_base() once the
// previous refresh has finished driving, so that a following differential 0xFC
// refresh diffs against what is actually on screen.
esp_err_t ssd1677_write_ram_partial(epd_device_t *dev, uint16_t x, uint16_t y,
                                     uint16_t w, uint16_t h, const uint8_t *data)
{
    epd_spi_t *spi = epd_get_spi(dev);

    // GDDRAM is read-only while the panel is refreshing; writes issued during
    // BUSY are ignored.
    WAIT(dev);

    ssd1677_set_ram_area(dev, x, y, w, h);

    CMD(spi, 0x24);
    epd_spi_write_data_bulk(spi, data, (w / 8) * h);

    return ESP_OK;
}

// Windowed base image (0x26) write. Used to commit the last partial window to
// the base plane after the panel finished driving, keeping 0x26 equal to the
// frame on screen so subsequent differential refreshes stay correct.
esp_err_t ssd1677_write_ram_partial_base(epd_device_t *dev, uint16_t x, uint16_t y,
                                          uint16_t w, uint16_t h, const uint8_t *data)
{
    epd_spi_t *spi = epd_get_spi(dev);

    WAIT(dev);

    ssd1677_set_ram_area(dev, x, y, w, h);

    CMD(spi, 0x26);
    epd_spi_write_data_bulk(spi, data, (w / 8) * h);

    return ESP_OK;
}

/*=============================================================================
 * Power Management
 *============================================================================*/

esp_err_t ssd1677_sleep(epd_device_t *dev)
{
    epd_spi_t *spi = epd_get_spi(dev);

    // The panel may be mid-refresh (async update): the power-down sequence
    // below is ignored while BUSY, so wait for idle before sending it.
    WAIT(dev);

    // The fast 0xFC sequence leaves the panel powered; drop the rails before
    // deep sleep so the panel idles at minimum power.
    CMD(spi, 0x22);  // Display update control
    DATA(spi, 0x03);  // ANALOG_OFF_PHASE | CLOCK_OFF
    CMD(spi, 0x20);  // Master activation
    WAIT(dev);

    CMD(spi, 0x10);  // Deep sleep
    DATA(spi, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Panel in deep sleep");
    return ESP_OK;
}

esp_err_t ssd1677_wake(epd_device_t *dev)
{
    return ssd1677_init(dev);
}
