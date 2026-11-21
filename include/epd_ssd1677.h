#pragma once

// SSD1677 controller (800x480 GDEM0397T81-class panel, Metalio E-Ink 4).
// Init sequence, waveform tables and plane handling follow the stock
// firmware (CloudZao/Metalio-E-INK4, MIT). Windows are written per RAM plane;
// refreshes are started asynchronously and finish with the BUSY interrupt.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "epd_bus.h"
#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_SSD1677_RAM_NEW 0x24    // "BW" RAM: the image to show
#define EPD_SSD1677_RAM_OLD 0x26    // "RED" RAM: the previous image (partial waveform reference)

typedef struct epd_ssd1677 epd_ssd1677_t;

typedef struct {
    epd_bus_t *bus;
    uint16_t pan_w, pan_h;      // native landscape panel: 800x480
    bool auto_old_plane;        // the chip copies 0x24 into 0x26 after a partial (hardware test, plan 2 Task 9)
} epd_ssd1677_cfg_t;

// Reset, run the init sequence and load the full-clean LUT.
esp_err_t epd_ssd1677_init(const epd_ssd1677_cfg_t *cfg, epd_ssd1677_t **out);
void epd_ssd1677_free(epd_ssd1677_t *c);
// Write one window of a packed 1 bpp plane into RAM 0x24 or 0x26. x and w must
// be multiples of 8; stride is the plane's bytes per panel row.
esp_err_t epd_ssd1677_write(epd_ssd1677_t *c, uint8_t ram, epd_rect_t panel_rect, const uint8_t *plane,
                            size_t stride);
// Load the mode's LUT if needed, start the refresh, return at once. done runs
// when BUSY drops (ISR context, or the caller's when the panel was already idle).
// GRAY expects 0x24 = LSB plane and 0x26 = MSB plane written before.
esp_err_t epd_ssd1677_start(epd_ssd1677_t *c, epd_mode_t mode, epd_busy_cb_t done, void *ctx);
// After a refresh finished: FULL_CLEAN sends the post-update option; PARTIAL
// and FAST rewrite the window into 0x26 (unless auto_old_plane); GRAY rewrites
// both planes with the black/white image so the next partial has the right base.
esp_err_t epd_ssd1677_after(epd_ssd1677_t *c, epd_mode_t mode, epd_rect_t panel_rect, const uint8_t *bw_plane,
                            size_t stride);
// The bus cannot read back (shared MOSI), so the temperature is not available.
esp_err_t epd_ssd1677_temperature(epd_ssd1677_t *c, int8_t *deg_c);
esp_err_t epd_ssd1677_deep_sleep(epd_ssd1677_t *c);   // 0x10 0x01: power off only; leaving it needs a reset
esp_err_t epd_ssd1677_reinit(epd_ssd1677_t *c);       // after a BUSY timeout: reset + init

// Partial LUT timing, for on-device tuning. In the 105-byte 0x32 table the
// timing lives in bytes 50..99 (10 groups x TP A, B, C, D, RP) and 100..104
// (frame rate, one nibble per group). The VS phase rows (bytes 0..49) and the
// voltages (0x03, 0x04, 0x2C) always stay stock.
typedef struct {
    uint8_t tp[10][4];  // groups 0..9: frames of phases A..D
    uint8_t rp[10];     // groups 0..9: repeat count
    uint8_t fr[5];      // frame rate bytes
} epd_ssd1677_timing_t;

// Current partial LUT timing.
void epd_ssd1677_partial_timing(const epd_ssd1677_t *c, epd_ssd1677_timing_t *out);
// Copy the stock partial table and patch only its timing bytes; the next
// partial/fast start loads it. NULL restores stock. Safety boundary: every TP
// and RP must be <= the stock value (shorter only), else ESP_ERR_INVALID_ARG
// and the current timing stays. Frame rate bytes are free. Kept across reinit.
esp_err_t epd_ssd1677_set_partial_timing(epd_ssd1677_t *c, const epd_ssd1677_timing_t *t);

#ifdef __cplusplus
}
#endif
