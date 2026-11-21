#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "epd_quant.h"

// Independent reference of the logical -> panel pixel mapping
static void ref_map(const epd_quant_t *q, int lx, int ly, int *px, int *py)
{
    int x, y;
    switch (q->rot) {
    case EPD_ROT_90:  x = ly;                y = q->pan_h - 1 - lx; break;
    case EPD_ROT_180: x = q->pan_w - 1 - lx; y = q->pan_h - 1 - ly; break;
    case EPD_ROT_270: x = q->pan_w - 1 - ly; y = lx;                break;
    default:          x = lx;                y = ly;                break;
    }
    if (q->mirror_x) x = q->pan_w - 1 - x;
    if (q->mirror_y) y = q->pan_h - 1 - y;
    *px = x;
    *py = y;
}

static int get_bit(const uint8_t *plane, size_t stride, int px, int py)
{
    return (plane[(size_t)py * stride + (size_t)(px >> 3)] >> (7 - (px & 7))) & 1;
}

static int rect_eq(epd_rect_t a, int x, int y, int w, int h)
{
    return a.x == x && a.y == y && a.w == w && a.h == h;
}

static void test_quant_map_rect_rot90(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 480, 800, 800, 480, EPD_ROT_90);
    // lx 0..7 -> py 479..472, ly 0..15 -> px 0..15
    CHECK(rect_eq(epd_quant_map_rect(&q, (epd_rect_t){0, 0, 8, 16}), 0, 472, 16, 8));
    CHECK(rect_eq(epd_quant_map_rect(&q, (epd_rect_t){0, 0, 480, 800}), 0, 0, 800, 480));
    // Clipped and empty inputs
    CHECK(rect_eq(epd_quant_map_rect(&q, (epd_rect_t){-8, -8, 1000, 1000}), 0, 0, 800, 480));
    epd_rect_t e = epd_quant_map_rect(&q, (epd_rect_t){480, 0, 8, 8});
    CHECK(e.w == 0 && e.h == 0);
}

static void test_quant_bw_threshold(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 8, 1, 8, 1, EPD_ROT_0);
    epd_quant_set_mode(&q, EPD_Q_BW, false);
    uint8_t l8[8] = {0, 127, 128, 255, 0, 0, 0, 0};
    uint8_t out = 0x55;
    epd_quant_rect(&q, l8, 8, (epd_rect_t){0, 0, 4, 1}, &out, NULL, 1);
    // bits 7..4 = 0,0,1,1; low nibble untouched (0x5)
    CHECK(out == 0x35);
}

static void test_quant_pure_values_never_dithered(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 16, 16, 16, 16, EPD_ROT_0);
    epd_quant_set_mode(&q, EPD_Q_BW_DITHER, false);
    uint8_t l8[256];
    uint8_t out[32];
    const epd_rect_t all = {0, 0, 16, 16};

    memset(l8, 0, sizeof(l8));
    memset(out, 0xFF, sizeof(out));
    epd_quant_rect(&q, l8, 16, all, out, NULL, 2);
    int ones = 0;
    for (int i = 0; i < 32; i++) ones += __builtin_popcount(out[i]);
    CHECK(ones == 0);

    memset(l8, 255, sizeof(l8));
    memset(out, 0, sizeof(out));
    epd_quant_rect(&q, l8, 16, all, out, NULL, 2);
    ones = 0;
    for (int i = 0; i < 32; i++) ones += __builtin_popcount(out[i]);
    CHECK(ones == 256);

    memset(l8, 128, sizeof(l8));
    memset(out, 0, sizeof(out));
    epd_quant_rect(&q, l8, 16, all, out, NULL, 2);
    ones = 0;
    for (int i = 0; i < 32; i++) ones += __builtin_popcount(out[i]);
    CHECK(ones >= 127 && ones <= 129);
    // Ordered, not a solid fill: every 4x4 tile has the same half-white mix
    int tile = 0;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) tile += get_bit(out, 2, x, y);
    CHECK(tile == 8);
}

static void test_quant_invert_lut(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 8, 1, 8, 1, EPD_ROT_0);
    epd_quant_set_mode(&q, EPD_Q_BW, true);
    CHECK(q.lut[0] == 1);
    CHECK(q.lut[255] == 0);
    uint8_t l8[8] = {0, 255, 0, 255, 0, 255, 0, 255};
    uint8_t out = 0;
    epd_quant_rect(&q, l8, 8, (epd_rect_t){0, 0, 8, 1}, &out, NULL, 1);
    CHECK(out == 0xAA);
    // Back to normal rebuilds the lut
    epd_quant_set_mode(&q, EPD_Q_BW, false);
    CHECK(q.lut[0] == 0 && q.lut[255] == 1);
}

static int roundtrip(epd_rot_t rot, bool mx, bool my, int lw, int lh, unsigned seed)
{
    int pw = (rot == EPD_ROT_90 || rot == EPD_ROT_270) ? lh : lw;
    int ph = (rot == EPD_ROT_90 || rot == EPD_ROT_270) ? lw : lh;
    epd_quant_t q;
    epd_quant_init(&q, (uint16_t)lw, (uint16_t)lh, (uint16_t)pw, (uint16_t)ph, rot);
    q.mirror_x = mx;
    q.mirror_y = my;
    size_t stride = (size_t)(pw + 7) / 8;
    uint8_t *l8 = malloc((size_t)lw * lh);
    uint8_t *pl = malloc(stride * ph);
    srand(seed);
    for (int i = 0; i < lw * lh; i++) l8[i] = (uint8_t)rand();
    memset(pl, 0, stride * ph);
    epd_quant_rect(&q, l8, (size_t)lw, (epd_rect_t){0, 0, (int16_t)lw, (int16_t)lh}, pl, NULL, stride);
    int bad = 0;
    for (int ly = 0; ly < lh; ly++) {
        for (int lx = 0; lx < lw; lx++) {
            int px, py;
            ref_map(&q, lx, ly, &px, &py);
            if (get_bit(pl, stride, px, py) != (l8[ly * lw + lx] >= 128)) bad++;
        }
    }
    free(l8);
    free(pl);
    return bad;
}

static void test_quant_rotation_roundtrip(void)
{
    CHECK(roundtrip(EPD_ROT_90, false, false, 480, 800, 1) == 0);
    // Every orientation and mirror combination on an odd-sized buffer
    for (int r = 0; r < 4; r++)
        for (int m = 0; m < 4; m++)
            CHECK(roundtrip((epd_rot_t)r, m & 1, (m >> 1) & 1, 13, 21, 2 + r * 4 + m) == 0);
}

static void test_quant_gray2_planes(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 8, 1, 8, 1, EPD_ROT_0);
    epd_quant_set_mode(&q, EPD_Q_GRAY2, false);
    uint8_t l8[8] = {0, 85, 170, 255, 42, 43, 213, 212};
    uint8_t lsb = 0, msb = 0;
    epd_quant_rect(&q, l8, 8, (epd_rect_t){0, 0, 8, 1}, &lsb, &msb, 1);
    // levels 0,1,2,3,0,1,3,2 -> MSB 0,0,1,1,0,0,1,1  LSB 0,1,0,1,0,1,1,0
    CHECK(msb == 0x33);
    CHECK(lsb == 0x56);
    epd_quant_set_mode(&q, EPD_Q_GRAY2, true);
    epd_quant_rect(&q, l8, 8, (epd_rect_t){0, 0, 8, 1}, &lsb, &msb, 1);
    CHECK(msb == 0xCC);
    CHECK(lsb == 0xA9);
}

static void test_quant_gray4_nibbles(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 4, 1, 4, 1, EPD_ROT_0);
    epd_quant_set_mode(&q, EPD_Q_GRAY4, false);
    uint8_t l8[4] = {0, 255, 0x80, 0x1F};
    uint8_t out[2] = {0, 0};
    epd_quant_rect(&q, l8, 4, (epd_rect_t){0, 0, 4, 1}, out, NULL, 2);
    CHECK(out[0] == 0x0F);
    CHECK(out[1] == 0x81);
    // A single odd pixel keeps the neighbour nibble
    l8[1] = 0;
    epd_quant_rect(&q, l8, 4, (epd_rect_t){1, 0, 1, 1}, out, NULL, 2);
    CHECK(out[0] == 0x00);
    epd_quant_set_mode(&q, EPD_Q_GRAY4, true);
    epd_quant_rect(&q, l8, 4, (epd_rect_t){0, 0, 1, 1}, out, NULL, 2);
    CHECK(out[0] == 0xF0);
}

// Quantize an all-white rect into an exactly-sized heap plane (ASan catches
// any overrun) and check that exactly the mapped bits flipped.
static int edge_case(epd_rect_t r)
{
    epd_quant_t q;
    epd_quant_init(&q, 480, 800, 800, 480, EPD_ROT_90);
    const size_t stride = 100;
    uint8_t *l8 = malloc(480 * 800);
    uint8_t *pl = malloc(stride * 480);
    uint8_t *ref = malloc(stride * 480);
    memset(l8, 255, 480 * 800);
    memset(pl, 0, stride * 480);
    memset(ref, 0, stride * 480);
    for (int ly = r.y; ly < r.y + r.h; ly++) {
        for (int lx = r.x; lx < r.x + r.w; lx++) {
            if (lx < 0 || ly < 0 || lx >= 480 || ly >= 800) continue;
            int px, py;
            ref_map(&q, lx, ly, &px, &py);
            ref[(size_t)py * stride + (size_t)(px >> 3)] |= (uint8_t)(0x80 >> (px & 7));
        }
    }
    epd_quant_rect(&q, l8, 480, r, pl, NULL, stride);
    int ok = memcmp(pl, ref, stride * 480) == 0;
    // Same with a black rect over an all-white plane
    memset(l8, 0, 480 * 800);
    memset(pl, 0xFF, stride * 480);
    for (size_t i = 0; i < stride * 480; i++) ref[i] = (uint8_t)~ref[i];
    epd_quant_rect(&q, l8, 480, r, pl, NULL, stride);
    ok = ok && memcmp(pl, ref, stride * 480) == 0;
    free(l8);
    free(pl);
    free(ref);
    return ok;
}

static void test_quant_rect_at_panel_edges(void)
{
    epd_quant_t q;
    epd_quant_init(&q, 480, 800, 800, 480, EPD_ROT_90);
    CHECK(rect_eq(epd_quant_map_rect(&q, (epd_rect_t){472, 792, 8, 8}), 792, 0, 8, 8));
    CHECK(rect_eq(epd_quant_map_rect(&q, (epd_rect_t){479, 799, 1, 1}), 799, 0, 1, 1));
    CHECK(edge_case((epd_rect_t){472, 792, 8, 8}));
    CHECK(edge_case((epd_rect_t){479, 799, 1, 1}));
    CHECK(edge_case((epd_rect_t){0, 0, 1, 1}));
    CHECK(edge_case((epd_rect_t){3, 5, 7, 11}));
    // Partly outside: clipped to the buffer, still no overrun
    CHECK(edge_case((epd_rect_t){470, 790, 10, 10}));
}

int main(void)
{
    test_quant_map_rect_rot90();
    test_quant_bw_threshold();
    test_quant_pure_values_never_dithered();
    test_quant_invert_lut();
    test_quant_rotation_roundtrip();
    test_quant_gray2_planes();
    test_quant_gray4_nibbles();
    test_quant_rect_at_panel_edges();
    TEST_DONE("quant");
}
