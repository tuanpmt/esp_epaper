// Exact bus bytes for known patterns, pinned to the vendor S31 encoding of the
// Metalio E-Ink 4 Plus firmware (read for behaviour only, not copied):
//  - pixel order: the S31 lookup puts the first pixel of every 4 in bits 7:6
//    (epdiy_riscv_ve_stub.c LUT4 for CONFIG_IDF_TARGET_ESP32S31: byte 0 of the
//    difference word indexes the 0x300 bank, which lut.c:313-319 fills with the
//    action << 6), and the vendor's own epd_pixel_order_selftest reads pixel p
//    as (out[p / 4] >> (6 - 2 * (p & 3))) & 3;
//  - code: 1 darkens, 2 lightens (lut.h:7-9 DARK_BYTE 0x55, CLEAR_BYTE 0xAA,
//    render_lcd.c:333-362 push colour 0 = dark, 1 = white).
// Every kernel of epd_wf must produce exactly these bytes.
#include <string.h>
#include "check.h"
#include "epd_wf.h"

enum { W = 64, H = 2, PS = W / 2, OS = W / 4 };

static _Alignas(8) uint8_t s_cur[PS * H], s_next[PS * H];
static _Alignas(8) uint16_t s_pp[PS * H];
static _Alignas(8) uint8_t s_tp[W * H];
static _Alignas(8) uint8_t s_out[OS * H], s_out2[OS * H];
static _Alignas(8) uint8_t s_lut[EPD_WF_PAIR_LUT_SIZE], s_lut2[EPD_WF_PAIR_LUT_SIZE];
static uint8_t s_tabs[4][256];

static void set_px(uint8_t *p, int x, int y, int v)
{
    uint8_t *b = &p[y * PS + x / 2];
    *b = (uint8_t)(x & 1 ? (*b & 0xF0) | v : (*b & 0x0F) | (v << 4));
}

// The vendor S31 byte for four pixel codes, pixel 0 first
static uint8_t stock_byte(int c0, int c1, int c2, int c3)
{
    return (uint8_t)((c0 << 6) | (c1 << 4) | (c2 << 2) | c3);
}

typedef enum { K_PHASE, K_PHASE2, K_PP, K_PP2, K_TP, K_COUNT } kernel_t;

// One phase of w over the whole W x H plane with the chosen kernel
static void run_kernel(kernel_t k, const epd_wf_phases_t *w, int phase, uint8_t *out)
{
    const epd_rect_t all = {0, 0, W, H};
    memset(out, 0xEE, OS * H);
    epd_wf_pair_lut(w, phase, s_lut);
    switch (k) {
    case K_PHASE:
        epd_wf_phase(s_lut, s_cur, s_next, PS, out, OS, all);
        break;
    case K_PHASE2:
        epd_wf_pair_lut(w, phase, s_lut2);
        epd_wf_phase2(s_lut, s_lut2, s_cur, s_next, PS, out, s_out2, OS, all);
        break;
    case K_PP:
        epd_wf_pair_plane(s_cur, s_next, PS, s_pp, all);
        epd_wf_phase_pp(s_lut, s_pp, PS, out, OS, all);
        break;
    case K_PP2:
        epd_wf_pair_lut(w, phase, s_lut2);
        epd_wf_pair_plane(s_cur, s_next, PS, s_pp, all);
        epd_wf_phase2_pp(s_lut, s_lut2, s_pp, PS, out, s_out2, OS, all);
        break;
    default:
        epd_wf_trans_plane(s_cur, s_next, PS, s_tp, all);
        epd_wf_phase_tables(w, phase, s_tabs);
        epd_wf_phase_tp(s_tabs, s_tp, W, out, OS, all);
        break;
    }
}

static const epd_wf_phases_t *table(epd_wf_kind_t k)
{
    return epd_wf_pick(&EPD_WF_E0470, k, EPD_WF_TEMP_DEFAULT);
}

// White page, one black pixel at x = 0, 1, 2, 3 (row 0) and x = 5 (row 1):
// DU phase 0 darkens white -> black, so exactly that pixel carries code 1
static void test_single_black_pixel(void)
{
    const epd_wf_phases_t *du = table(EPD_WF_DU);
    CHECK(epd_wf_action(du, 0, 15, 0) == 1);
    for (int k = 0; k < K_COUNT; k++) {
        for (int x = 0; x < 4; x++) {
            memset(s_cur, 0xFF, sizeof(s_cur));
            memset(s_next, 0xFF, sizeof(s_next));
            set_px(s_next, x, 0, 0);
            set_px(s_next, 5, 1, 0);
            run_kernel((kernel_t)k, du, 0, s_out);
            const int c[4] = {x == 0, x == 1, x == 2, x == 3};
            CHECK(s_out[0] == stock_byte(c[0], c[1], c[2], c[3]));
            CHECK(s_out[0] == (uint8_t)(0x40 >> (2 * x)));    // 0x40, 0x10, 0x04, 0x01
            for (int b = 1; b < OS; b++) {
                CHECK(s_out[b] == 0);
            }
            CHECK(s_out[OS + 1] == 0x10);   // x = 5: second pixel of byte 1
            CHECK(s_out[OS] == 0);
        }
    }
}

// GC16 white -> black and black -> white: every phase's bytes over a uniform
// area are the stock fill bytes of the table's code (0x55 darken, 0xAA lighten)
static void test_gc16_uniform_codes(void)
{
    const epd_wf_phases_t *gc = table(EPD_WF_GC16);
    for (int k = 0; k < K_COUNT; k++) {
        for (int dir = 0; dir < 2; dir++) {
            const int from = dir ? 0 : 15, to = dir ? 15 : 0;
            int last = 0;
            for (int p = 0; p < gc->phases; p++) {
                memset(s_cur, from * 0x11, sizeof(s_cur));
                memset(s_next, to * 0x11, sizeof(s_next));
                run_kernel((kernel_t)k, gc, p, s_out);
                const int a = epd_wf_action(gc, p, from, to);
                const uint8_t want = a == 1 ? 0x55 : (a == 2 ? 0xAA : 0x00);
                for (int b = 0; b < OS * H; b++) {
                    CHECK(s_out[b] == want);
                }
                last = a ? a : last;
            }
            CHECK(last == (to == 0 ? 1 : 2));   // ends darkening to black, lightening to white
        }
    }
}

// Mixed codes in one byte: the four pixels of a group take four different
// transitions, so a mirrored or rotated order cannot pass
static void test_mixed_group(void)
{
    const epd_wf_phases_t *du = table(EPD_WF_DU);
    // px0 white -> black (1), px1 black -> white (2), px2 white -> white (0), px3 black -> black (DU: hold 0 in phase 0)
    for (int k = 0; k < K_COUNT; k++) {
        memset(s_cur, 0xFF, sizeof(s_cur));
        memset(s_next, 0xFF, sizeof(s_next));
        set_px(s_cur, 0, 0, 15);
        set_px(s_next, 0, 0, 0);
        set_px(s_cur, 1, 0, 0);
        set_px(s_next, 1, 0, 15);
        set_px(s_cur, 3, 0, 0);
        set_px(s_next, 3, 0, 0);
        run_kernel((kernel_t)k, du, 0, s_out);
        const int c3 = epd_wf_action(du, 0, 0, 0);
        CHECK(s_out[0] == stock_byte(1, 2, 0, c3));
        CHECK(s_out[0] == 0x60);
    }
}

int main(void)
{
    test_single_black_pixel();
    test_gc16_uniform_codes();
    test_mixed_group();
    TEST_DONE("bus_bytes");
}
