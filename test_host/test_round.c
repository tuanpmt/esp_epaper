#include "check.h"
#include "epd_round.h"

// 3.97in: logical 480x800 portrait, panel 800x480, ROT_90: the panel byte axis
// (panel x) is the logical y axis, so rounding aligns logical y and h.
static void test_round_area_aligns_panel_byte_axis(void)
{
    const epd_rect_t a = {10, 13, 11, 8};   // (10,13)-(20,20)
    const epd_rect_t r = epd_round_rect(a, 480, 800, EPD_ROT_90, 8, 0, 0);
    CHECK(r.y % 8 == 0 && r.h % 8 == 0);
    CHECK(r.y <= 13 && r.y + r.h >= 21);
    CHECK(r.x == 10 && r.w == 11);          // the other axis is untouched without padding
    const epd_rect_t p = epd_round_rect(a, 480, 800, EPD_ROT_90, 8, 24, 12);
    CHECK(p.x == 0 && p.x + p.w >= 10 + 11 + 24);
    CHECK(p.y % 8 == 0 && p.y <= 13 - 12);
}

static void test_round_area_clips_to_screen(void)
{
    const epd_rect_t a = {470, 790, 20, 20};
    const epd_rect_t r = epd_round_rect(a, 480, 800, EPD_ROT_90, 8, 24, 12);
    CHECK(r.x + r.w <= 480 && r.y + r.h <= 800);
    CHECK(r.y % 8 == 0 && r.h % 8 == 0);
    const epd_rect_t z = epd_round_rect((epd_rect_t){500, 900, 5, 5}, 480, 800, EPD_ROT_90, 8, 0, 0);
    CHECK(z.w <= 0 || z.h <= 0);
}

static void test_round_area_rot0_aligns_x(void)
{
    const epd_rect_t r = epd_round_rect((epd_rect_t){3, 5, 6, 7}, 800, 480, EPD_ROT_0, 8, 0, 0);
    CHECK(r.x == 0 && r.w == 16 && r.y == 5 && r.h == 7);
}

int main(void)
{
    test_round_area_aligns_panel_byte_axis();
    test_round_area_clips_to_screen();
    test_round_area_rot0_aligns_x();
    TEST_DONE("round");
}
