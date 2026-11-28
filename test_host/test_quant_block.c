// The block path of epd_quant_rect (90 / 270 rotations) against the row walk
// it replaced (quant_ref.c, esp_epaper 4747dcf): whole planes byte for byte,
// bytes outside the rect included, for the formats of both recab panels
// (3.97in BW dither 1 bpp, Plus GRAY4 4 bpp), GRAY2, odd sizes and padded
// strides, mirrors, inversion and rects partly outside the buffer.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "epd_quant.h"
#include "quant_ref.h"

static uint32_t s_rng = 1;

static uint32_t rnd(void)
{
    s_rng = s_rng * 1103515245u + 12345u;
    return s_rng >> 8;
}

typedef struct {
    int lw, lh, pw, ph;
    size_t l8_stride, pan_stride;
    epd_rot_t rot;
    bool mx, my;
    epd_qmode_t mode;
    bool invert;
    size_t l8_off;      // L8 buffer this many bytes past an aligned address
} qcase_t;

static size_t plane_stride_min(epd_qmode_t m, int pw)
{
    return m == EPD_Q_GRAY4 ? (size_t)(pw + 1) / 2 : (size_t)(pw + 7) / 8;
}

// Runs one rect through both paths from identical planes; 0 when identical
static int run_rect(const qcase_t *c, const uint8_t *l8, epd_rect_t r, uint8_t *a0, uint8_t *a1, uint8_t *b0,
                    uint8_t *b1, size_t plane_bytes)
{
    epd_quant_t q;
    epd_quant_init(&q, (uint16_t)c->lw, (uint16_t)c->lh, (uint16_t)c->pw, (uint16_t)c->ph, c->rot);
    q.mirror_x = c->mx;
    q.mirror_y = c->my;
    epd_quant_set_mode(&q, c->mode, c->invert);
    for (size_t i = 0; i < plane_bytes; i++) {
        a0[i] = b0[i] = (uint8_t)rnd();
        a1[i] = b1[i] = (uint8_t)rnd();
    }
    epd_quant_rect(&q, l8, c->l8_stride, r, a0, a1, c->pan_stride);
    ref_quant_rect(&q, l8, c->l8_stride, r, b0, b1, c->pan_stride);
    for (size_t i = 0; i < plane_bytes; i++) {
        if (a0[i] != b0[i] || a1[i] != b1[i]) {
            printf("  %dx%d->%dx%d st %zu/%zu rot %d mx %d my %d mode %d inv %d rect %d,%d,%d,%d: byte %zu "
                   "got %02x/%02x want %02x/%02x\n",
                   c->lw, c->lh, c->pw, c->ph, c->l8_stride, c->pan_stride, (int)c->rot, c->mx, c->my, (int)c->mode,
                   c->invert, r.x, r.y, r.w, r.h, i, a0[i], a1[i], b0[i], b1[i]);
            return 1;
        }
    }
    return 0;
}

// Full frame, edge rects and `random_rects` random rects (some partly outside)
static int run_case(const qcase_t *c, int random_rects)
{
    const size_t l8_bytes = c->l8_stride * (size_t)c->lh;
    const size_t plane_bytes = c->pan_stride * (size_t)c->ph;
    uint8_t *l8_mem = malloc(l8_bytes + 4);
    uint8_t *l8 = l8_mem + c->l8_off;
    uint8_t *a0 = malloc(plane_bytes), *a1 = malloc(plane_bytes);
    uint8_t *b0 = malloc(plane_bytes), *b1 = malloc(plane_bytes);
    CHECK(l8 && a0 && a1 && b0 && b1);
    for (size_t i = 0; i < l8_bytes; i++) {
        // mostly UI-like levels (0, 255, a few grays) with noise, so every mode sees edges
        const uint32_t v = rnd();
        l8[i] = (v & 3) == 0 ? (uint8_t)(v >> 4) : ((v & 4) ? 255 : 0);
    }
    int bad = 0;
    const epd_rect_t fixed[] = {
        {0, 0, (int16_t)c->lw, (int16_t)c->lh}, {0, 0, 1, 1}, {(int16_t)(c->lw - 1), (int16_t)(c->lh - 1), 1, 1},
        {1, 3, 7, 9}, {0, (int16_t)(c->lh / 2), (int16_t)c->lw, 1}, {(int16_t)(c->lw / 2), 0, 1, (int16_t)c->lh},
        {-5, -7, (int16_t)(c->lw + 20), 40}, {(int16_t)(c->lw - 3), (int16_t)(c->lh - 3), 30, 30},
    };
    for (size_t k = 0; k < sizeof(fixed) / sizeof(fixed[0]) && !bad; k++) {
        bad += run_rect(c, l8, fixed[k], a0, a1, b0, b1, plane_bytes);
    }
    for (int k = 0; k < random_rects && !bad; k++) {
        const int x = (int)(rnd() % (uint32_t)(c->lw + 8)) - 4, y = (int)(rnd() % (uint32_t)(c->lh + 8)) - 4;
        const int w = 1 + (int)(rnd() % (uint32_t)c->lw), h = 1 + (int)(rnd() % (uint32_t)c->lh);
        bad += run_rect(c, l8, (epd_rect_t){(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h}, a0, a1, b0, b1,
                        plane_bytes);
    }
    free(l8_mem);
    free(a0);
    free(a1);
    free(b0);
    free(b1);
    return bad;
}

// 3.97in: L8 480x800 portrait, SSD1677 800x480, BW dither (and plain BW,
// and the GRAY2 planes of a gray refresh); LVGL may pad the L8 rows
static void test_block_eink4_formats(void)
{
    const epd_qmode_t modes[] = {EPD_Q_BW_DITHER, EPD_Q_BW, EPD_Q_GRAY2};
    const size_t l8_strides[] = {480, 496};
    const size_t pan_pads[] = {0, 3};
    int bad = 0;
    for (size_t m = 0; m < 3; m++) {
        for (int inv = 0; inv < 2; inv++) {
            for (size_t s = 0; s < 2; s++) {
                for (size_t p = 0; p < 2; p++) {
                    const qcase_t c = {480, 800, 800, 480, l8_strides[s], 100 + pan_pads[p],
                                       p ? EPD_ROT_270 : EPD_ROT_90, s == 1, p == 1, modes[m], inv != 0, 0};
                    bad += run_case(&c, 12);
                }
            }
        }
    }
    CHECK(bad == 0);
}

// Plus: L8 684x1216 portrait, E0470 1216x684, GRAY4
static void test_block_plus_formats(void)
{
    const size_t l8_strides[] = {684, 688};
    const size_t pan_strides[] = {608, 611};
    int bad = 0;
    for (int inv = 0; inv < 2; inv++) {
        for (size_t s = 0; s < 2; s++) {
            for (size_t p = 0; p < 2; p++) {
                const qcase_t c = {684, 1216, 1216, 684, l8_strides[s], pan_strides[p],
                                   p ? EPD_ROT_270 : EPD_ROT_90, s == 1, false, EPD_Q_GRAY4, inv != 0, 0};
                bad += run_case(&c, 8);
            }
        }
    }
    CHECK(bad == 0);
}

// Odd sizes (not multiples of 8 or of the block), every rotation and
// mirroring, every mode, padded strides
static void test_block_odd_sizes(void)
{
    const int dims[][2] = {{37, 53}, {1, 1}, {9, 70}, {65, 33}, {100, 129}};
    const epd_rot_t rots[] = {EPD_ROT_0, EPD_ROT_90, EPD_ROT_180, EPD_ROT_270};
    const epd_qmode_t modes[] = {EPD_Q_BW, EPD_Q_BW_DITHER, EPD_Q_GRAY2, EPD_Q_GRAY4};
    int bad = 0;
    for (size_t d = 0; d < sizeof(dims) / sizeof(dims[0]); d++) {
        for (size_t r = 0; r < 4; r++) {
            for (int mir = 0; mir < 4; mir++) {
                for (size_t m = 0; m < 4; m++) {
                    const int lw = dims[d][0], lh = dims[d][1];
                    const bool swap = rots[r] == EPD_ROT_90 || rots[r] == EPD_ROT_270;
                    const int pw = swap ? lh : lw, ph = swap ? lw : lh;
                    const qcase_t c = {lw, lh, pw, ph, (size_t)lw + (size_t)(mir & 1) * 5,
                                       plane_stride_min(modes[m], pw) + (size_t)(mir >> 1) * 2, rots[r],
                                       (mir & 1) != 0, (mir & 2) != 0, modes[m], (m & 1) != 0, 0};
                    bad += run_case(&c, 6);
                }
            }
        }
    }
    CHECK(bad == 0);
}

// The word path needs a 4-byte aligned L8 buffer and stride: otherwise the
// block path, same bytes
static void test_block_unaligned_l8(void)
{
    const qcase_t a = {480, 800, 800, 480, 480, 100, EPD_ROT_90, false, false, EPD_Q_BW_DITHER, false, 1};
    const qcase_t b = {684, 1216, 1216, 684, 686, 608, EPD_ROT_90, false, false, EPD_Q_GRAY4, true, 2};
    const qcase_t c = {480, 800, 800, 480, 481, 100, EPD_ROT_270, true, true, EPD_Q_BW, false, 0};
    CHECK(run_case(&a, 4) == 0);
    CHECK(run_case(&b, 4) == 0);
    CHECK(run_case(&c, 4) == 0);
}

// Rotations 0 / 180 keep the row walk (already sequential in L8)
static void test_block_rot0_unchanged(void)
{
    const qcase_t c = {800, 480, 800, 480, 800, 100, EPD_ROT_0, false, false, EPD_Q_BW_DITHER, false, 0};
    const qcase_t d = {800, 480, 800, 480, 800, 100, EPD_ROT_180, true, false, EPD_Q_BW, true, 0};
    CHECK(run_case(&c, 4) == 0);
    CHECK(run_case(&d, 4) == 0);
}

int main(void)
{
    test_block_eink4_formats();
    test_block_plus_formats();
    test_block_odd_sizes();
    test_block_unaligned_l8();
    test_block_rot0_unchanged();
    TEST_DONE("test_quant_block");
}
