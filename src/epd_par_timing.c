// Parallel panel line and frame timing (see epd_par_timing.h).

#include <stddef.h>
#include "epd_par_timing.h"

// E0470A14-AF figures as used by the vendor firmware of the Metalio E-Ink 4
// Plus (board config and LCD driver comments; CloudZao Metalio-E-INK4-Plus,
// MIT): 20 MHz XCL, XLE 9 clocks, porch 4, >= 10 clocks after the data,
// CKV low >= 1 us, about 6 start/tail lines, 85 Hz phases.
const epd_par_panel_t EPD_PANEL_E0470 = {
    .pclk_hz = 20000000,
    .ckv_tick_hz = 10000000,
    .width_px = 1216,
    .rows = 684,
    .le_high_clk = 9,
    .front_porch_clk = 4,
    .min_post_clk = 10,
    .ckv_low_ns = 1000,
    .extra_lines = 6,
    .phase_hz = 85,
};

static uint32_t div_up(uint32_t a, uint32_t b)
{
    return (a + b - 1) / b;
}

esp_err_t epd_par_timing_calc(const epd_par_panel_t *p, epd_par_timing_t *out)
{
    if (!p || !out || !p->pclk_hz || !p->ckv_tick_hz || !p->rows || !p->phase_hz ||
        p->width_px == 0 || p->width_px % 4 || p->pclk_hz % p->ckv_tick_hz || p->ckv_low_ns < 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint32_t ppt = p->pclk_hz / p->ckv_tick_hz;     // pixel clocks per CKV tick
    epd_par_timing_t t = {0};
    t.line_bytes = (uint16_t)(p->width_px / 4);
    t.data_clk = (uint16_t)(p->le_high_clk + p->front_porch_clk + t.line_bytes);
    const uint32_t low = div_up((uint32_t)p->ckv_low_ns * (p->ckv_tick_hz / 1000u), 1000000u);
    uint32_t high = div_up(t.data_clk, ppt);
    const uint32_t need = (uint32_t)t.data_clk + p->min_post_clk;
    if ((high + low) * ppt < need) {
        high += div_up(need - (high + low) * ppt, ppt);
    }
    t.ckv_high_ticks = (uint16_t)high;
    t.ckv_low_ticks = (uint16_t)low;
    t.ckv_period_ticks = (uint16_t)(high + low);
    t.line_clk = (uint16_t)(t.ckv_period_ticks * ppt);
    t.h_front = (uint16_t)(t.line_clk - t.data_clk);
    t.line_ns = (uint32_t)((uint64_t)t.line_clk * 1000000000u / p->pclk_hz);
    t.frame_lines = (uint16_t)(p->rows + p->extra_lines);
    t.frame_us = (uint32_t)((uint64_t)t.frame_lines * t.line_ns / 1000u);
    t.phase_us = 1000000u / p->phase_hz;
    *out = t;
    return t.frame_us < t.phase_us ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
