// Waveform engine: pair LUT, phase bytes, band limits, commit, table choice.
#include <stdlib.h>
#include <string.h>
#include "check.h"
#include "epd_wf.h"

// 2-phase synthetic table: phase 0 darkens towards 0, phase 1 lightens towards 15
static uint8_t s_tab[2][16][4];
static epd_wf_phases_t s_w = {.phases = 2, .lut = &s_tab[0][0][0]};

static void set_act(int f, int to, int from, int a)
{
    const int sh = 6 - 2 * (from % 4);
    s_tab[f][to][from / 4] = (uint8_t)((s_tab[f][to][from / 4] & ~(3 << sh)) | (a << sh));
}

static void make_table(void)
{
    memset(s_tab, 0, sizeof(s_tab));
    for (int from = 0; from < 16; from++) {
        if (from != 0) {
            set_act(0, 0, from, 1);
        }
        if (from != 15) {
            set_act(1, 15, from, 2);
        }
    }
    set_act(0, 7, 3, 2);    // an odd one to catch from/to swaps
}

static uint8_t s_lut[EPD_WF_PAIR_LUT_SIZE];

static void test_wf_pair_lut_matches_action(void)
{
    make_table();
    CHECK(epd_wf_action(&s_w, 0, 3, 7) == 2);
    CHECK(epd_wf_action(&s_w, 0, 7, 3) == 0);
    for (int f = 0; f < 2; f++) {
        epd_wf_pair_lut(&s_w, f, s_lut);
        for (int c = 0; c < 256; c++) {
            for (int n = 0; n < 256; n++) {
                const int left = epd_wf_action(&s_w, f, c >> 4, n >> 4);
                const int right = epd_wf_action(&s_w, f, c & 15, n & 15);
                CHECK(s_lut[(c << 8) | n] == (left | (right << 2)));
            }
        }
    }
    // the real DU table too
    const epd_wf_phases_t *du = &EPD_WF_E0470.ranges[0].w[EPD_WF_DU];
    epd_wf_pair_lut(du, 0, s_lut);
    CHECK(s_lut[(0xFF << 8) | 0x00] == (1 | (1 << 2)));   // white -> black: darken both
    CHECK(s_lut[(0x00 << 8) | 0xFF] == (2 | (2 << 2)));   // black -> white: lighten both
}

// 8 px row, 4 bpp: pixel order inside the bus byte
static void test_wf_phase_bit_order(void)
{
    make_table();
    epd_wf_pair_lut(&s_w, 0, s_lut);    // phase 0: x -> 0 darkens (1)
    // cur: all white. next: px0 black, px1 white, px2 white, px3 black, px4..7 white
    const uint8_t cur[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t next[4] = {0x0F, 0xF0, 0xFF, 0xFF};
    uint8_t out[2] = {0xAA, 0xAA};
    epd_wf_phase(s_lut, cur, next, 4, out, 2, (epd_rect_t){0, 0, 8, 1});
    CHECK(out[0] == ((1 << 0) | (1 << 6)));   // px0 bits 1:0, px3 bits 7:6
    CHECK(out[1] == 0x00);
}

static void test_wf_phase_band_only(void)
{
    enum { W = 64, H = 6, PS = W / 2, OS = W / 4 };
    make_table();
    epd_wf_pair_lut(&s_w, 0, s_lut);
    uint8_t cur[PS * H], next[PS * H], out[OS * H];
    memset(cur, 0xFF, sizeof(cur));
    memset(next, 0x00, sizeof(next));   // every pixel wants black
    memset(out, 0xCC, sizeof(out));     // sentinel
    const epd_rect_t band = {16, 2, 24, 3};
    epd_wf_phase(s_lut, cur, next, PS, out, OS, band);
    for (int y = 0; y < H; y++) {
        for (int b = 0; b < OS; b++) {
            const bool in = y >= 2 && y < 5 && b >= 16 / 4 && b < (16 + 24) / 4;
            CHECK(out[y * OS + b] == (in ? 0x55 : 0xCC));
        }
    }
}

static void test_wf_commit_band_only(void)
{
    enum { W = 32, H = 4, PS = W / 2 };
    uint8_t cur[PS * H], next[PS * H];
    memset(cur, 0x11, sizeof(cur));
    memset(next, 0x99, sizeof(next));
    epd_wf_commit(cur, next, PS, (epd_rect_t){8, 1, 8, 2});
    for (int y = 0; y < H; y++) {
        for (int b = 0; b < PS; b++) {
            const bool in = y >= 1 && y < 3 && b >= 4 && b < 8;
            CHECK(cur[y * PS + b] == (in ? 0x99 : 0x11));
        }
    }
}

static void test_wf_band_align(void)
{
    epd_rect_t r = epd_wf_band_align((epd_rect_t){5, 3, 3, 10}, 1216, 684);
    CHECK(r.x == 4 && r.w == 4 && r.y == 3 && r.h == 10);
    r = epd_wf_band_align((epd_rect_t){1210, 680, 20, 20}, 1216, 684);
    CHECK(r.x == 1208 && r.w == 8 && r.y == 680 && r.h == 4);
    r = epd_wf_band_align((epd_rect_t){-3, -2, 7, 5}, 1216, 684);
    CHECK(r.x == 0 && r.w == 4 && r.y == 0 && r.h == 3);
    r = epd_wf_band_align((epd_rect_t){10, 10, 0, 5}, 1216, 684);
    CHECK(r.w == 0 || r.h == 0);
}

static void test_wf_pick_nearest_range(void)
{
    static const epd_wf_range_t ranges[2] = {
        {.t_min = 0, .t_max = 20, .w = {[EPD_WF_DU] = {.phases = 30, .lut = &s_tab[0][0][0]}}},
        {.t_min = 21, .t_max = 50, .w = {[EPD_WF_DU] = {.phases = 20, .lut = &s_tab[0][0][0]}}},
    };
    const epd_wf_set_t set = {.n = 2, .ranges = ranges};
    CHECK(epd_wf_pick(&set, EPD_WF_DU, -5)->phases == 30);
    CHECK(epd_wf_pick(&set, EPD_WF_DU, 10)->phases == 30);
    CHECK(epd_wf_pick(&set, EPD_WF_DU, 30)->phases == 20);
    CHECK(epd_wf_pick(&set, EPD_WF_DU, 70)->phases == 20);
    CHECK(epd_wf_pick(&set, EPD_WF_GC16, 30) == NULL);   // no such table
    CHECK(epd_wf_pick(&EPD_WF_E0470, EPD_WF_GC16, 25)->phases == 48);
}

static void test_wf_gating_cold(void)
{
    CHECK(epd_wf_kind_for(EPD_M_FAST, 25, true) == EPD_WF_DU_SHORT);
    CHECK(epd_wf_kind_for(EPD_M_PARTIAL, 25, true) == EPD_WF_DU);
    CHECK(epd_wf_kind_for(EPD_M_GRAY, 25, true) == EPD_WF_GL16);
    CHECK(epd_wf_kind_for(EPD_M_FULL_CLEAN, 25, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_FAST, 4, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_PARTIAL, 46, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_FAST, 5, true) == EPD_WF_DU_SHORT);
    CHECK(epd_wf_kind_for(EPD_M_FAST, -40, false) == EPD_WF_DU_SHORT);   // no reading: 25 C
    CHECK(epd_wf_kind_for(EPD_M_GRAY, 2, true) == EPD_WF_GL16);
}

// Reference: one bus byte at a time (the first, plain implementation)
static void ref_phase(const uint8_t *lut, const uint8_t *cur, const uint8_t *next, size_t ps, uint8_t *out, size_t os,
                      epd_rect_t b)
{
    for (int y = b.y; y < b.y + b.h; y++) {
        for (int i = 0; i < b.w / 4; i++) {
            const uint8_t *c = cur + (size_t)y * ps + b.x / 2 + 2 * i;
            const uint8_t *n = next + (size_t)y * ps + b.x / 2 + 2 * i;
            out[(size_t)y * os + b.x / 4 + i] = (uint8_t)(lut[(c[0] << 8) | n[0]] | (lut[(c[1] << 8) | n[1]] << 4));
        }
    }
}

enum { RW = 1216, RH = 24, RPS = RW / 2, ROS = RW / 4 };
static uint8_t r_cur[RPS * RH], r_next[RPS * RH], r_a[ROS * RH], r_b[ROS * RH], r_c[ROS * RH];
static uint8_t s_lut2[EPD_WF_PAIR_LUT_SIZE];

static void test_wf_phase_words_match_bytes(void)
{
    srand(7);
    for (size_t i = 0; i < sizeof(r_cur); i++) {
        r_cur[i] = (uint8_t)rand();
        r_next[i] = (uint8_t)rand();
    }
    const epd_wf_phases_t *gc = &EPD_WF_E0470.ranges[0].w[EPD_WF_GC16];
    epd_wf_pair_lut(gc, 20, s_lut);
    // every head/tail alignment of the 32-bit loop, plus full rows
    const epd_rect_t bands[] = {{0, 0, RW, RH}, {4, 1, 1204, 5}, {8, 2, 20, 3}, {12, 0, 4, 2}, {1212, 3, 4, 4},
                                {28, 5, 1000, 9}, {0, 0, 8, 1}};
    for (size_t k = 0; k < sizeof(bands) / sizeof(bands[0]); k++) {
        memset(r_a, 0x5A, sizeof(r_a));
        memset(r_b, 0x5A, sizeof(r_b));
        epd_wf_phase(s_lut, r_cur, r_next, RPS, r_a, ROS, bands[k]);
        ref_phase(s_lut, r_cur, r_next, RPS, r_b, ROS, bands[k]);
        CHECK(memcmp(r_a, r_b, sizeof(r_a)) == 0);
    }
}

static void test_wf_phase2_matches_two_calls(void)
{
    const epd_wf_phases_t *gc = &EPD_WF_E0470.ranges[0].w[EPD_WF_GC16];
    epd_wf_pair_lut(gc, 10, s_lut);
    epd_wf_pair_lut(gc, 11, s_lut2);
    const epd_rect_t b = {4, 2, 1208, 20};
    memset(r_a, 0, sizeof(r_a));
    memset(r_b, 0, sizeof(r_b));
    memset(r_c, 0, sizeof(r_c));
    epd_wf_phase2(s_lut, s_lut2, r_cur, r_next, RPS, r_a, r_b, ROS, b);
    ref_phase(s_lut, r_cur, r_next, RPS, r_c, ROS, b);
    CHECK(memcmp(r_a, r_c, sizeof(r_a)) == 0);
    memset(r_c, 0, sizeof(r_c));
    ref_phase(s_lut2, r_cur, r_next, RPS, r_c, ROS, b);
    CHECK(memcmp(r_b, r_c, sizeof(r_b)) == 0);
}

static void test_wf_phase_same(void)
{
    for (int k = 0; k < EPD_WF_COUNT; k++) {
        const epd_wf_phases_t *w = &EPD_WF_E0470.ranges[0].w[k];
        for (int f = 1; f < w->phases; f++) {
            const bool same = memcmp(w->lut + (size_t)f * 64, w->lut + (size_t)(f - 1) * 64, 64) == 0;
            CHECK(epd_wf_phase_same(w, f, f - 1) == same);
        }
    }
    const epd_wf_phases_t *du = &EPD_WF_E0470.ranges[0].w[EPD_WF_DU_FAST];
    CHECK(epd_wf_phase_same(du, 1, 0));     // DU fast repeats its first phase
}

static uint16_t r_pp[RPS * RH];

static void test_wf_pair_plane_kernel_matches(void)
{
    const epd_wf_phases_t *gc = &EPD_WF_E0470.ranges[0].w[EPD_WF_GC16];
    epd_wf_pair_lut(gc, 20, s_lut);
    epd_wf_pair_lut(gc, 21, s_lut2);
    const epd_rect_t bands[] = {{0, 0, RW, RH}, {4, 1, 1204, 5}, {8, 2, 20, 3}, {12, 0, 4, 2}, {1212, 3, 4, 4},
                                {28, 5, 1000, 9}};
    for (size_t k = 0; k < sizeof(bands) / sizeof(bands[0]); k++) {
        memset(r_pp, 0xEE, sizeof(r_pp));
        epd_wf_pair_plane(r_cur, r_next, RPS, r_pp, bands[k]);
        memset(r_a, 0x5A, sizeof(r_a));
        memset(r_b, 0x5A, sizeof(r_b));
        epd_wf_phase_pp(s_lut, r_pp, RPS, r_a, ROS, bands[k]);
        ref_phase(s_lut, r_cur, r_next, RPS, r_b, ROS, bands[k]);
        CHECK(memcmp(r_a, r_b, sizeof(r_a)) == 0);
        memset(r_a, 0x5A, sizeof(r_a));
        memset(r_c, 0x5A, sizeof(r_c));
        epd_wf_phase2_pp(s_lut, s_lut2, r_pp, RPS, r_a, r_c, ROS, bands[k]);
        CHECK(memcmp(r_a, r_b, sizeof(r_a)) == 0);
        memset(r_b, 0x5A, sizeof(r_b));
        ref_phase(s_lut2, r_cur, r_next, RPS, r_b, ROS, bands[k]);
        CHECK(memcmp(r_c, r_b, sizeof(r_c)) == 0);
    }
}

static uint8_t r_tp[RW * RH];
static uint8_t s_tabs[4][256];

static void test_wf_trans_plane_kernel_matches(void)
{
    const epd_wf_phases_t *gc = &EPD_WF_E0470.ranges[0].w[EPD_WF_GC16];
    epd_wf_pair_lut(gc, 20, s_lut);
    epd_wf_phase_tables(gc, 20, s_tabs);
    const epd_rect_t bands[] = {{0, 0, RW, RH}, {4, 1, 1204, 5}, {8, 2, 20, 3}, {12, 0, 4, 2}, {1212, 3, 4, 4},
                                {28, 5, 1000, 9}};
    for (size_t k = 0; k < sizeof(bands) / sizeof(bands[0]); k++) {
        memset(r_tp, 0xEE, sizeof(r_tp));
        epd_wf_trans_plane(r_cur, r_next, RPS, r_tp, bands[k]);
        memset(r_a, 0x5A, sizeof(r_a));
        memset(r_b, 0x5A, sizeof(r_b));
        epd_wf_phase_tp(s_tabs, r_tp, RW, r_a, ROS, bands[k]);
        ref_phase(s_lut, r_cur, r_next, RPS, r_b, ROS, bands[k]);
        CHECK(memcmp(r_a, r_b, sizeof(r_a)) == 0);
    }
    // pixel 0 of a row: high nibble of cur byte 0 -> high nibble of tp byte 0
    epd_wf_trans_plane(r_cur, r_next, RPS, r_tp, bands[0]);
    CHECK(r_tp[0] == (uint8_t)((r_cur[0] & 0xF0) | (r_next[0] >> 4)));
    CHECK(r_tp[1] == (uint8_t)(((r_cur[0] & 0x0F) << 4) | (r_next[0] & 0x0F)));
}

int main(void)
{
    test_wf_pair_lut_matches_action();
    test_wf_phase_bit_order();
    test_wf_phase_band_only();
    test_wf_commit_band_only();
    test_wf_band_align();
    test_wf_pick_nearest_range();
    test_wf_gating_cold();
    test_wf_phase_words_match_bytes();
    test_wf_phase2_matches_two_calls();
    test_wf_phase_same();
    test_wf_pair_plane_kernel_matches();
    test_wf_trans_plane_kernel_matches();
    TEST_DONE("wf");
}
