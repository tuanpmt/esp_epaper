#pragma once

// Line and frame timing of a parallel e-paper panel driven by an LCD-style
// peripheral (pixel clock XCL, latch XLE, start pulse XSTL per line) plus a
// gate clock CKV from a timer peripheral. Pure arithmetic, host-tested.
//
// One line: XLE high, front porch, the data bytes, then dummy clocks until
// the CKV period ends. CKV is high while the line's data moves and low for
// at least ckv_low_ns. The panel figures (E0470A14-AF datasheet section
// 6.3.1, as quoted by the vendor firmware of the Metalio E-Ink 4 Plus) give
// a 16.9 us line and an 11.7 ms frame; the vendor waveforms are tuned for
// one phase per 1/85 s, so the run loop paces phases to phase_us.

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pclk_hz;           // XCL
    uint32_t ckv_tick_hz;       // CKV timer resolution
    uint16_t width_px;          // source pixels per line (2 bpp, 4 per 8-bit clock)
    uint16_t rows;              // gate rows
    uint8_t le_high_clk;        // XLE high, in pixel clocks
    uint8_t front_porch_clk;    // after XLE, before the data
    uint8_t min_post_clk;       // dummy clocks required after the data
    uint16_t ckv_low_ns;        // CKV low time per line, panel minimum 1000 ns
    uint16_t extra_lines;       // start and tail lines per frame
    uint8_t phase_hz;           // waveform phase rate
} epd_par_panel_t;

typedef struct {
    uint16_t line_bytes;        // data clocks of one line
    uint16_t data_clk;          // XLE + porch + data
    uint16_t line_clk;          // whole line, multiple of the CKV tick
    uint16_t h_front;           // dummy clocks after the data
    uint16_t ckv_high_ticks, ckv_low_ticks, ckv_period_ticks;
    uint16_t frame_lines;       // rows + extra_lines
    uint32_t line_ns, frame_us, phase_us;
} epd_par_timing_t;

// ESP_ERR_INVALID_ARG: zero or inconsistent fields, CKV low under 1 us;
// ESP_ERR_INVALID_SIZE: a frame does not fit in one phase.
esp_err_t epd_par_timing_calc(const epd_par_panel_t *p, epd_par_timing_t *out);

// E0470A14-AF / ED047TC2, 1216 x 684, as on the Metalio E-Ink 4 Plus
extern const epd_par_panel_t EPD_PANEL_E0470;

#ifdef __cplusplus
}
#endif
