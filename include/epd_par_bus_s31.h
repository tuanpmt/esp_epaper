#pragma once

// ESP32-S31 scan bus for parallel e-paper panels (implements epd_par_bus.h).
//
// LCD_CAM in RGB mode through the IDF low-level layer: XCL = PCLK,
// XLE = HSYNC, XSTL = DE, D0..D7 on the LCD data pads (IOMUX when they are
// the LCD_DATA pads), one frame per phase buffer from PSRAM by AXI-GDMA.
// CKV comes from an RMT TX channel looping one symbol (high, low) for
// frame_lines + 1 periods; SPV is a GPIO.
//
// Frame start (IRAM, critical section): SPV (idle high) low, CKV loop started, wait for
// the 2nd CKV rising edge (the gate chain took SPV low on the 1st), SPV high,
// wait for the 2nd falling edge, start the LCD. The lead time from the RMT
// start to that rising edge is measured; a frame whose lead is not about one
// line is refused (the edges would be one line off).
//
// Pins are routed to the peripherals only while a frame runs; between frames
// every pad is a GPIO output at its idle level (SPV, XSTL high: active low;
// the rest low). route_pins = false keeps every signal
// inside the chip (stage B1 self-check with no panel activity at all): the
// LCD and the RMT still run and are timed.
//
// Own code against ESP-IDF v6.1.1 (Apache-2.0) LL headers; the clock, trans
// buffer and one-shot start sequence follow the IDF RGB panel driver. Signal
// roles follow the vendor firmware of the Metalio E-Ink 4 Plus.

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "epd_par_bus.h"
#include "epd_par_timing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int data0;                  // D0; D1..D7 follow (GPIO8..15 on the Plus)
    int xcl, xle, xstl, ckv, spv, xoe, mode;
    bool route_pins;            // false: nothing reaches the pads (B1 internal check)
    bool drive_oe;              // raise XOE and MODE at the first frame (panel driving, rails on)
    bool xle_active_low;        // default: XLE (HSYNC) active high
    bool xstl_active_high;      // default: XSTL (DE) active low
    bool xcl_out_neg;           // data changes on the XCL falling edge instead of the rising one
    uint8_t v_back;             // blank lines after VSYNC before the first data row
    uint8_t v_front;            // blank lines after the last row; 1 + v_back + rows + v_front = frame_lines
    uint16_t start_delay_ns;    // extra delay from the CKV falling edge to the LCD start
} epd_s31_bus_cfg_t;

typedef struct {
    uint32_t frames, underruns, timeouts, sync_errors;
    uint32_t last_frame_us;     // LCD start to VSYNC end
    int32_t last_rmt_lag_us;    // CKV loop end minus LCD frame end (should be about one line or less)
    uint32_t last_lead_ns;      // RMT start to the 2nd CKV rising edge (pins mode; about one line)
    uint32_t last_dma_eofs;     // DMA EOF events in the last frame (1)
} epd_s31_bus_stats_t;

// bufs: the two phase buffers (line_bytes x rows each, 64-byte aligned, PSRAM or internal)
esp_err_t epd_s31_bus_new(const epd_s31_bus_cfg_t *cfg, const epd_par_panel_t *panel, const epd_par_timing_t *t,
                          uint8_t *const bufs[2], epd_par_bus_t *out);
void epd_s31_bus_del(void);
void epd_s31_bus_stats(epd_s31_bus_stats_t *st);
// Write the CPU cache back to memory for rows [y0, y0 + rows) of a phase
// buffer, after computing it and before its scan (DMA reads memory)
esp_err_t epd_s31_bus_flush(const uint8_t *buf, uint16_t y0, uint16_t rows);

#ifdef __cplusplus
}
#endif
