#pragma once

// ESP32-S31 scan bus for parallel e-paper panels (implements epd_par_bus.h).
//
// LCD_CAM in RGB mode through the IDF low-level layer: XCL = PCLK,
// XLE = HSYNC, XSTL = DE, D0..D7 on the LCD_DATA IOMUX pads, one frame per
// phase buffer from PSRAM by AXI-GDMA. CKV from RMT channel 1; SPV a GPIO.
//
// Mirrors the vendor firmware (display/output_lcd/lcd_driver.c, behaviour
// only): HSYNC position 1, no HSYNC in the porch, vertical (1, 0, rows, 1),
// RMT channel 1 at 10 MHz looping (high, low) for rows + 5 periods, and the
// frame start in one critical section: SPV low, CKV start, one CKV period in
// whole microseconds (17), SPV high, the CKV high time in whole microseconds
// (15), LCD start.
//
// Pins are routed to the peripherals at the first frame of a refresh and stay
// routed (as stock) until park() or a failed frame; outside a refresh every
// pad is a GPIO output at the idle level of the LCD output (SPV and XSTL
// high: active low; the rest low). route_pins = false keeps every
// signal inside the chip (stage B1 self-check): the LCD, the DMA and the RMT
// still run and are timed.
//
// Own code against the ESP-IDF v6.1.1 (Apache-2.0) LL headers.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "epd_par_bus.h"
#include "epd_par_timing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int data0;                  // D0; D1..D7 follow; must be the LCD_DATA IOMUX pads (GPIO8..15)
    int xcl, xle, xstl, ckv, spv, xoe, mode;
    bool route_pins;            // false: nothing reaches the pads (B1 internal check)
    bool drive_oe;              // raise XOE and MODE at each frame (panel driving, rails on)
} epd_s31_bus_cfg_t;

typedef struct {
    uint32_t frames, underruns, timeouts, sync_errors;
    uint32_t last_frame_us;     // LCD start to the VSYNC_END interrupt
    uint32_t last_eof_us;       // LCD start to the DMA EOF: the last data byte entered the LCD
    int32_t last_ckv_margin_us; // nominal CKV end (rows + 5 periods) minus the DMA EOF: > 0 = CKV still clocks
    uint32_t last_lead_ns;      // CKV start to LCD start: one CKV period + the CKV high time (32 us)
    uint32_t last_dma_eofs;     // DMA EOF events in the last frame (1)
    uint32_t last_start_us;     // scan_start entry to LCD start (includes the 17 + 15 us lead)
    uint32_t last_tail_us;      // VSYNC_END interrupt to scan_wait return
} epd_s31_bus_stats_t;

// bufs: the two phase buffers (line_bytes x rows each, 64-byte aligned, PSRAM or internal)
esp_err_t epd_s31_bus_new(const epd_s31_bus_cfg_t *cfg, const epd_par_panel_t *panel, const epd_par_timing_t *t,
                          uint8_t *const bufs[2], epd_par_bus_t *out);
// Switch between the internal check and the panel (parks the bus first)
void epd_s31_bus_set_pins(bool route_pins, bool drive_oe);
void epd_s31_bus_stats(epd_s31_bus_stats_t *st);
// Write the CPU cache back to memory for rows [y0, y0 + rows) of a phase
// buffer, after computing it and before its scan (DMA reads memory)
esp_err_t epd_s31_bus_flush(const uint8_t *buf, uint16_t y0, uint16_t rows);

#ifdef __cplusplus
}
#endif
