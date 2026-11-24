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

// A second patchable copy of the stock partial table, used by EPD_M_FAST
// (interaction) only; PARTIAL and TRANSITION use the partial table. Same
// rule: TP/RP shorter than stock, frame rate free, NULL restores stock.
// While both tables hold the same bytes, switching between them reloads nothing.
esp_err_t epd_ssd1677_set_fast_timing(epd_ssd1677_t *c, const epd_ssd1677_timing_t *t);
void epd_ssd1677_fast_timing(const epd_ssd1677_t *c, epd_ssd1677_timing_t *out);

// Timing candidates of the 3.97in LUT sweep (plan 9 Task 8), numbered as the
// black squares the sweep draws: 1 stock (561 ms), 2 FR 0x44 (391 ms),
// 3 FR 0x44 + TP A of group 0 = 6, 4 FR 0x55, 5 FR 0x55 + TP A of group 0 = 6.
// epd_ssd1677_init loads the picks of ssd1677_lut.c into both tables.
#define EPD_SSD1677_LUT_CANDIDATES 5
esp_err_t epd_ssd1677_lut_candidate(uint8_t id, epd_ssd1677_timing_t *out);

// Full partial tables per refresh role (plan 9 Task 12): page turns
// (EPD_M_PARTIAL), screen changes (EPD_M_TRANSITION), buttons (EPD_M_FAST).
// Tables 1..5 equal the timing candidates above; 6..14 add TP A8 and change
// the VS rows of groups 0 and 1 (a "deepen" phase after the main drive, a
// text table that also re-drives white, a direct-drive A2 table). Every
// table passes epd_ssd1677_table_check. 13 and 14 are the deepen phase at
// slower frame rates (FR 0x33, stock 0x22): stronger erase for screen changes. The timing setters above set PAGE
// and SCREEN (partial) or BUTTON (fast) and mark the role's table id 0.
#define EPD_SSD1677_LUT_SIZE 112
#define EPD_SSD1677_LUT_TABLES 14
typedef enum {
    EPD_SSD1677_ROLE_PAGE,
    EPD_SSD1677_ROLE_SCREEN,
    EPD_SSD1677_ROLE_BUTTON,
    EPD_SSD1677_ROLE_COUNT
} epd_ssd1677_role_t;
esp_err_t epd_ssd1677_table(uint8_t id, uint8_t out[EPD_SSD1677_LUT_SIZE]);
const char *epd_ssd1677_table_name(uint8_t id);     // NULL for an unknown id
// Safety envelope against the stock partial table: bytes 105..111 and the
// VCOM row unchanged, groups 2..9 silent, per group no more frames than
// stock and no repeat, frame rate not slower than stock
esp_err_t epd_ssd1677_table_check(const uint8_t *lut);
// Takes effect at the next start of that role (call from the task that owns c)
esp_err_t epd_ssd1677_use_table(epd_ssd1677_t *c, epd_ssd1677_role_t role, uint8_t id);
uint8_t epd_ssd1677_table_in_use(const epd_ssd1677_t *c, epd_ssd1677_role_t role);   // 0: set by timing

#ifdef __cplusplus
}
#endif
