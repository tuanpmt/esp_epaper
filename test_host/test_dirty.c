#include <stdbool.h>
#include <stdint.h>

#include "check.h"
#include "epd_dirty.h"

static epd_rect_t R(int x, int y, int w, int h)
{
    epd_rect_t r = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
    return r;
}

static bool rect_eq(epd_rect_t a, int x, int y, int w, int h)
{
    return a.x == x && a.y == y && a.w == w && a.h == h;
}

// True when every pixel of `in` lies in some rect of `set` (pixel-wise check)
static bool covered(epd_rect_t in, const epd_rect_t *set, uint8_t n)
{
    for (int y = in.y; y < in.y + in.h; y++) {
        for (int x = in.x; x < in.x + in.w; x++) {
            bool hit = false;
            for (uint8_t i = 0; i < n && !hit; i++) {
                hit = x >= set[i].x && x < set[i].x + set[i].w &&
                      y >= set[i].y && y < set[i].y + set[i].h;
            }
            if (!hit) {
                return false;
            }
        }
    }
    return true;
}

static void test_dirty_clip_and_align_edges(void)
{
    epd_dirty_t d;
    epd_rect_t out[EPD_DIRTY_MAX];

    epd_dirty_init(&d, 800, 480, 8, 60);
    epd_dirty_add(&d, R(-5, 10, 20, 10));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 10, 16, 10));

    epd_dirty_add(&d, R(795, 470, 20, 20));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 792, 470, 8, 10));

    epd_dirty_add(&d, R(10, 10, 0, 10));
    epd_dirty_add(&d, R(10, 10, 10, -3));
    epd_dirty_add(&d, R(900, 0, 10, 10));
    epd_dirty_add(&d, R(0, 480, 10, 10));
    epd_dirty_add(&d, R(-20, -20, 20, 20));
    CHECK(d.n == 0);
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 0);

    // Extreme int16 values must not overflow
    epd_dirty_add(&d, R(32767, 32767, 32767, 32767));
    CHECK(d.n == 0);
    epd_dirty_add(&d, R(-32768, -32768, 32767, 32767));
    CHECK(d.n == 0);
    epd_dirty_add(&d, R(-32768, 100, 32767, 1));   // ends at x=-1: outside
    CHECK(d.n == 0);
    epd_dirty_add(&d, R(-30000, 0, 30003, 1));         // ends at x=3
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 0, 8, 1));

    // Panel width not a multiple of align: aligned x end clamps to the panel
    epd_dirty_init(&d, 250, 122, 8, 60);
    epd_dirty_add(&d, R(245, 0, 5, 3));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 240, 0, 10, 3));

    // 4 bpp alignment (2)
    epd_dirty_init(&d, 800, 480, 2, 60);
    epd_dirty_add(&d, R(3, 3, 2, 2));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 2, 3, 4, 2));
}

static void test_dirty_merge_overlap(void)
{
    epd_dirty_t d;
    epd_rect_t out[EPD_DIRTY_MAX];

    epd_dirty_init(&d, 800, 480, 8, 60);
    epd_dirty_add(&d, R(0, 0, 16, 16));
    epd_dirty_add(&d, R(8, 8, 16, 16));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 0, 24, 24));

    // Edge-adjacent rects merge
    epd_dirty_add(&d, R(0, 0, 16, 16));
    epd_dirty_add(&d, R(16, 0, 16, 16));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 0, 32, 16));

    // Far apart rects stay separate
    epd_dirty_add(&d, R(0, 0, 16, 16));
    epd_dirty_add(&d, R(100, 100, 16, 16));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 2);

    // A new rect bridging two existing ones collapses all three
    epd_dirty_add(&d, R(0, 0, 16, 16));
    epd_dirty_add(&d, R(64, 0, 16, 16));
    epd_dirty_add(&d, R(8, 0, 64, 4));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 0, 80, 16));
}

static void test_dirty_cap_to_full(void)
{
    epd_dirty_t d;
    epd_rect_t out[EPD_DIRTY_MAX];

    epd_dirty_init(&d, 800, 480, 8, 60);
    // 3 disjoint rects, 3 * 800*100 = 240000 > 0.6 * 384000 = 230400
    epd_dirty_add(&d, R(0, 0, 800, 100));
    epd_dirty_add(&d, R(0, 150, 800, 100));
    CHECK(!epd_dirty_is_full(&d));
    epd_dirty_add(&d, R(0, 300, 800, 100));
    CHECK(epd_dirty_is_full(&d));
    epd_dirty_add(&d, R(5, 5, 5, 5));   // no-op once full
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 0, 800, 480));
    CHECK(!epd_dirty_is_full(&d));

    epd_dirty_add_all(&d);
    CHECK(epd_dirty_is_full(&d));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 1);
    CHECK(rect_eq(out[0], 0, 0, 800, 480));
}

static void test_dirty_overflow_merges(void)
{
    epd_dirty_t d;
    epd_rect_t in[20], out[EPD_DIRTY_MAX];

    epd_dirty_init(&d, 800, 480, 8, 60);
    for (int i = 0; i < 20; i++) {
        in[i] = R((i % 5) * 160, (i / 5) * 120, 16, 8);
        epd_dirty_add(&d, in[i]);
        CHECK(d.n <= EPD_DIRTY_MAX);
    }
    CHECK(!epd_dirty_is_full(&d));
    uint8_t n = d.n;
    CHECK(n >= 1 && n <= EPD_DIRTY_MAX);
    for (int i = 0; i < 20; i++) {
        CHECK(covered(in[i], d.r, d.n));
    }
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == n);

    // take with a smaller max merges down instead of dropping rects
    for (int i = 0; i < 20; i++) {
        epd_dirty_add(&d, in[i]);
    }
    n = epd_dirty_take(&d, out, 3);
    CHECK(n >= 1 && n <= 3);
    for (int i = 0; i < 20; i++) {
        CHECK(covered(in[i], out, n));
    }
}

static void test_dirty_take_clears(void)
{
    epd_dirty_t d;
    epd_rect_t out[EPD_DIRTY_MAX];

    epd_dirty_init(&d, 800, 480, 8, 60);
    epd_dirty_add(&d, R(0, 0, 8, 8));
    epd_dirty_add(&d, R(100, 100, 8, 8));
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 2);
    CHECK(d.n == 0);
    CHECK(epd_dirty_take(&d, out, EPD_DIRTY_MAX) == 0);
    CHECK(d.pan_w == 800 && d.pan_h == 480 && d.align == 8 && d.full_area_pct == 60);
    epd_dirty_add(&d, R(0, 0, 8, 8));
    CHECK(d.n == 1);
}

int main(void)
{
    test_dirty_clip_and_align_edges();
    test_dirty_merge_overlap();
    test_dirty_cap_to_full();
    test_dirty_overflow_merges();
    test_dirty_take_clears();
    TEST_DONE("dirty");
}
