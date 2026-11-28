// Parallel panel timing: the E0470 numbers of the vendor firmware.
#include "check.h"
#include "epd_par_timing.h"

static void test_timing_e0470_line_and_frame(void)
{
    epd_par_timing_t t;
    CHECK(epd_par_timing_calc(&EPD_PANEL_E0470, &t) == ESP_OK);
    CHECK(t.line_bytes == 304);
    CHECK(t.data_clk == 317);
    CHECK(t.ckv_high_ticks == 159 && t.ckv_low_ticks == 10 && t.ckv_period_ticks == 169);
    CHECK(t.line_clk == 338);
    CHECK(t.h_front == 21 && t.h_front >= EPD_PANEL_E0470.min_post_clk);
    CHECK(t.line_ns == 16900);
    CHECK(t.frame_lines == 690);
    CHECK(t.frame_us >= 11600 && t.frame_us <= 11700);
    CHECK(t.phase_us == 11764);
    CHECK(t.frame_us < t.phase_us);
}

static void test_timing_rejects_short_ckv_low(void)
{
    epd_par_panel_t p = EPD_PANEL_E0470;
    epd_par_timing_t t;
    p.ckv_low_ns = 900;
    CHECK(epd_par_timing_calc(&p, &t) == ESP_ERR_INVALID_ARG);
    p = EPD_PANEL_E0470;
    p.width_px = 1214;      // not a whole number of bus bytes
    CHECK(epd_par_timing_calc(&p, &t) == ESP_ERR_INVALID_ARG);
    p = EPD_PANEL_E0470;
    p.ckv_tick_hz = 3000000;    // pclk is not a multiple of the tick
    CHECK(epd_par_timing_calc(&p, &t) == ESP_ERR_INVALID_ARG);
}

static void test_timing_post_clocks_stretch_the_line(void)
{
    epd_par_panel_t p = EPD_PANEL_E0470;
    epd_par_timing_t t;
    p.min_post_clk = 40;    // more dummy clocks than the CKV low time gives
    p.phase_hz = 75;        // the longer line still fits a phase
    CHECK(epd_par_timing_calc(&p, &t) == ESP_OK);
    CHECK(t.h_front >= 40);
    CHECK(t.line_clk == t.ckv_period_ticks * 2);
    CHECK(t.ckv_low_ticks == 10);
}

static void test_timing_rejects_frame_longer_than_phase(void)
{
    epd_par_panel_t p = EPD_PANEL_E0470;
    epd_par_timing_t t;
    p.pclk_hz = 10000000;   // half speed: 23 ms frame > 11.76 ms phase
    p.ckv_tick_hz = 10000000;
    CHECK(epd_par_timing_calc(&p, &t) == ESP_ERR_INVALID_SIZE);
}

int main(void)
{
    test_timing_e0470_line_and_frame();
    test_timing_rejects_short_ckv_low();
    test_timing_post_clocks_stretch_the_line();
    test_timing_rejects_frame_longer_than_phase();
    TEST_DONE("par_timing");
}
