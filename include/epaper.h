#ifndef _EPAPER_H_
#define _EPAPER_H_

#include <stdint.h>
#include <stdbool.h>
#include "epaper_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle
typedef struct epd_device* epd_handle_t;

// Panel info (read-only after init)
typedef struct {
    uint16_t width;
    uint16_t height;
    uint32_t buffer_size;       // In bytes
    epd_color_mode_t color_mode;
} epd_panel_info_t;

/**
 * @brief Initialize e-paper display
 *
 * When config.framebuffer_enable is false, the internal full-frame buffer is
 * not allocated and epd_update() streams the caller's buffer straight to the
 * panel. epd_fill(), epd_set_base_image(), epd_lvgl_flush() and
 * epd_flush_framebuffer() then return ESP_ERR_NOT_SUPPORTED, and
 * epd_get_framebuffer() returns NULL.
 *
 * @param config Device configuration
 * @param handle Output handle
 * @return ESP_OK on success
 */
esp_err_t epd_init(const epd_config_t *config, epd_handle_t *handle);

/**
 * @brief Deinitialize e-paper display
 */
esp_err_t epd_deinit(epd_handle_t handle);

/**
 * @brief Get panel information
 */
esp_err_t epd_get_info(epd_handle_t handle, epd_panel_info_t *info);

/**
 * @brief Clear display (full white)
 */
esp_err_t epd_clear(epd_handle_t handle);

/**
 * @brief Fill display with color (0x00=black, 0xFF=white)
 */
esp_err_t epd_fill(epd_handle_t handle, uint8_t color);

/**
 * @brief Update full screen with buffer
 * @param buffer Pixel data (1 bit per pixel, MSB first)
 * @param mode Update mode (full/fast/partial)
 */
esp_err_t epd_update(epd_handle_t handle, const uint8_t *buffer, epd_update_mode_t mode);

/**
 * @brief Stream buffer to the panel and trigger a refresh without waiting
 *
 * Same flow as epd_update(), but returns as soon as the master activation has
 * been issued. The panel keeps driving (BUSY high) for the refresh duration,
 * so the caller must epd_wait_busy() before the next command sequence, or rely
 * on the controller's wait-before-use guards (e.g. ssd1677_write_ram()) to
 * synchronize at the next access. The caller's buffer is only read during the
 * synchronous SPI transfer, so it is free to reuse once this returns.
 *
 * @param buffer Pixel data (1 bit per pixel, MSB first)
 * @param mode Update mode (full/fast/partial)
 */
esp_err_t epd_update_async(epd_handle_t handle, const uint8_t *buffer, epd_update_mode_t mode);

/**
 * @brief Partial update
 * @param x Start X (must be multiple of 8)
 * @param y Start Y
 * @param w Width (must be multiple of 8)
 * @param h Height
 * @param buffer Pixel data
 */
esp_err_t epd_update_partial(epd_handle_t handle, uint16_t x, uint16_t y, 
                              uint16_t w, uint16_t h, const uint8_t *buffer);

/**
 * @brief Check if the panel supports windowed partial updates
 *
 * True when the panel advertises EPD_CAP_PARTIAL and its controller implements
 * windowed RAM writes. Does not imply a base image RAM exists; see
 * epd_write_base_partial().
 */
bool epd_supports_partial(epd_handle_t handle);

/**
 * @brief Check if the panel exposes a base image RAM (see epd_write_base_partial())
 */
bool epd_supports_base_partial(epd_handle_t handle);

/**
 * @brief Stream a byte-aligned rectangle into the display RAM without refreshing
 *
 * Writes the rectangle into the current image RAM (0x24) only. The caller owns
 * the base plane synchronization: after the panel finishes driving, the same
 * rectangle must be committed via epd_write_base_partial() on controllers with
 * a differential refresh sequence, so that the next refresh diffs against what
 * is actually on screen.
 *
 * @param x Start X (must be multiple of 8)
 * @param y Start Y
 * @param w Width (must be multiple of 8)
 * @param h Height
 * @param buffer Pixel data (1 bit per pixel, MSB first)
 */
esp_err_t epd_write_partial(epd_handle_t handle, uint16_t x, uint16_t y,
                            uint16_t w, uint16_t h, const uint8_t *buffer);

/**
 * @brief Stream a byte-aligned rectangle into the base image RAM
 *
 * Optional: only supported when the controller implements a base image plane
 * (e.g. SSD1677 0x26). See epd_write_partial() for the synchronization contract.
 *
 * @param x Start X (must be multiple of 8)
 * @param y Start Y
 * @param w Width (must be multiple of 8)
 * @param h Height
 * @param buffer Pixel data (1 bit per pixel, MSB first)
 */
esp_err_t epd_write_base_partial(epd_handle_t handle, uint16_t x, uint16_t y,
                                 uint16_t w, uint16_t h, const uint8_t *buffer);

/**
 * @brief Trigger a refresh of the previously streamed image without waiting
 *
 * Trigger-only counterpart to epd_update(): assumes the panel RAM already holds
 * the frame to display (via epd_update(), epd_write_partial(), etc.) and issues
 * the master activation, returning before the panel finishes driving. Call
 * epd_wait_busy() before the next command sequence, or rely on the
 * controller's wait-before-use guards to synchronize at the next access.
 *
 * @param mode Update mode (full/fast/partial)
 */
esp_err_t epd_update_partial_async(epd_handle_t handle, epd_update_mode_t mode);

/**
 * @brief Set base image for partial updates
 */
esp_err_t epd_set_base_image(epd_handle_t handle, const uint8_t *buffer);

/**
 * @brief Enter deep sleep mode
 */
esp_err_t epd_sleep(epd_handle_t handle);

/**
 * @brief Wake from deep sleep
 */
esp_err_t epd_wake(epd_handle_t handle);

/**
 * @brief Check if display is busy
 */
bool epd_is_busy(epd_handle_t handle);

/**
 * @brief Wait until display is ready
 * @param timeout_ms Timeout in milliseconds (0 = infinite)
 */
esp_err_t epd_wait_busy(epd_handle_t handle, uint32_t timeout_ms);

// ============ LVGL Integration Interface ============

/**
 * @brief Flush callback for LVGL
 * Call this from lv_display_set_flush_cb
 * 
 * Usage in LVGL 9:
 *   lv_display_t *disp = lv_display_create(width, height);
 *   lv_display_set_flush_cb(disp, my_flush_cb);
 *   lv_display_set_user_data(disp, epd_handle);
 * 
 * In my_flush_cb:
 *   epd_handle_t h = lv_display_get_user_data(disp);
 *   epd_lvgl_flush(h, area, px_map);
 *   lv_display_flush_ready(disp);
 */
esp_err_t epd_lvgl_flush(epd_handle_t handle, const void *area, const uint8_t *px_map);

/**
 * @brief Get framebuffer for direct LVGL rendering
 * @return Pointer to internal framebuffer, or NULL when the internal
 *         framebuffer is disabled or the handle is invalid
 */
uint8_t* epd_get_framebuffer(epd_handle_t handle);

/**
 * @brief Flush framebuffer to display
 */
esp_err_t epd_flush_framebuffer(epd_handle_t handle, epd_update_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // _EPAPER_H_
