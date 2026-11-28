#pragma once

// Scan bus of a parallel e-paper panel: pushes one phase buffer (2 bpp, one
// line of line_bytes per gate row) through the panel with the timing of
// epd_par_timing. The chip-specific implementation (ESP32-S31: LCD_CAM in
// RGB mode via LL plus RMT for CKV) lives next to the board; the run loop
// (epd_par_run) only sees these operations.

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *ctx;
    // Start one frame: SPV pulse, CKV loop, XOE/MODE high at the first line.
    esp_err_t (*scan_start)(void *ctx, const uint8_t *phase_buf);
    // Wait for the frame end; on timeout or DMA underrun drop XOE/MODE and fail.
    esp_err_t (*scan_wait)(void *ctx, uint32_t timeout_us);
    // All lines low, peripheral idle (between refreshes, before sleep).
    void (*park)(void *ctx);
} epd_par_bus_t;

#ifdef __cplusplus
}
#endif
