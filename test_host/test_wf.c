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
                CHECK(s_lut[(c << 8) | n] == ((left << 2) | right));   // left pixel in the higher bits
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
    // cur: all white. next: px0 black, px6 black, the rest white
    const uint8_t cur[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t next[4] = {0x0F, 0xFF, 0xFF, 0x0F};
    uint8_t out[2] = {0xAA, 0xAA};
    epd_wf_phase(s_lut, cur, next, 4, out, 2, (epd_rect_t){0, 0, 8, 1});
    CHECK(out[0] == (1 << 6));   // vendor S31 order: px0 in bits 7:6
    CHECK(out[1] == (1 << 2));   // px6 = px2 of byte 1: bits 3:2
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
    // FAST: DU 14 (stock "Fast" profile); PARTIAL and TRANSITION: DU 20
    CHECK(epd_wf_kind_for(EPD_M_FAST, 25, true) == EPD_WF_DU_FAST);
    CHECK(epd_wf_kind_for(EPD_M_PARTIAL, 25, true) == EPD_WF_DU);
    CHECK(epd_wf_kind_for(EPD_M_GRAY, 25, true) == EPD_WF_GL16);
    CHECK(epd_wf_kind_for(EPD_M_FULL_CLEAN, 25, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_FAST, 4, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_PARTIAL, 46, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_FAST, 5, true) == EPD_WF_DU_FAST);
    CHECK(epd_wf_kind_for(EPD_M_FAST, -40, false) == EPD_WF_DU_FAST);   // no reading: 25 C, as stock always
    CHECK(epd_wf_kind_for(EPD_M_TRANSITION, 46, true) == EPD_WF_GC16);
    CHECK(epd_wf_kind_for(EPD_M_GRAY, 2, true) == EPD_WF_GL16);
}

// A transition (screen change without a flash) is a differential DU, as a partial
static void test_wf_transition_is_du(void)
{
    CHECK(epd_wf_kind_for(EPD_M_TRANSITION, 25, true) == EPD_WF_DU);
    CHECK(epd_wf_kind_for(EPD_M_TRANSITION, 4, true) == EPD_WF_GC16);
    CHECK(epd_wf_rule_for(EPD_M_TRANSITION, EPD_WF_DU) == EPD_WF_SKIP_SAME_KEEP_BLACK);
}

// Reference: one bus byte at a time (the first, plain implementation)
static void ref_phase(const uint8_t *lut, const uint8_t *cur, const uint8_t *next, size_t ps, uint8_t *out, size_t os,
                      epd_rect_t b)
{
    for (int y = b.y; y < b.y + b.h; y++) {
        for (int i = 0; i < b.w / 4; i++) {
            const uint8_t *c = cur + (size_t)y * ps + b.x / 2 + 2 * i;
            const uint8_t *n = next + (size_t)y * ps + b.x / 2 + 2 * i;
            out[(size_t)y * os + b.x / 4 + i] = (uint8_t)((lut[(c[0] << 8) | n[0]] << 4) | lut[(c[1] << 8) | n[1]]);
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


// Mode -> table for the fast UI (plan 9 T5)
static void test_wf_kind_for_modes(void)
{
    CHECK(epd_wf_kind_for(EPD_M_FAST, 25, false) == EPD_WF_DU_FAST);
    CHECK(epd_wf_kind_for(EPD_M_PARTIAL, 25, false) == EPD_WF_DU);
    CHECK(epd_wf_kind_for(EPD_M_TRANSITION, 25, false) == EPD_WF_DU);
    CHECK(epd_wf_kind_for(EPD_M_GRAY, 25, false) == EPD_WF_GL16);
    CHECK(epd_wf_kind_for(EPD_M_FULL_CLEAN, 25, false) == EPD_WF_GC16);
    CHECK(epd_wf_kind_binary(EPD_WF_DU) && epd_wf_kind_binary(EPD_WF_DU_FAST) && epd_wf_kind_binary(EPD_WF_DU_SHORT));
    CHECK(!epd_wf_kind_binary(EPD_WF_GL16) && !epd_wf_kind_binary(EPD_WF_GC16));
}

// GRAY is a differential GL16 over the changed box (vendor gray.differential)
static void test_wf_rule_gray_differential(void)
{
    CHECK(epd_wf_rule_for(EPD_M_GRAY, EPD_WF_GL16) == EPD_WF_SKIP_SAME_GL16);
    CHECK(epd_wf_rule_for(EPD_M_GRAY, EPD_WF_GL16_TRIM) == EPD_WF_SKIP_SAME_GL16);
    CHECK(epd_wf_rule_for(EPD_M_FULL_CLEAN, EPD_WF_GC16) == EPD_WF_DRIVE_ALL);
    CHECK(epd_wf_rule_for(EPD_M_FAST, EPD_WF_DU_FAST) == EPD_WF_SKIP_SAME_KEEP_BLACK);
}

// Reference for the binary target of one pixel: an unchanged level stays
// (nothing is driven), a changed one goes to black or white
static int bin_px(int cur, int next)
{
    return next == cur ? cur : (next >= 8 ? 15 : 0);
}

static uint8_t pxb(const uint8_t *p, size_t ps, int x, int y)
{
    const uint8_t b = p[(size_t)y * ps + x / 2];
    return (uint8_t)(x & 1 ? b & 15 : b >> 4);
}

static void test_wf_du_commit_binarizes(void)
{
    enum { W = 16, H = 2, PS = W / 2 };
    uint8_t cur[PS * H], tgt[PS * H], g[PS * H];
    memset(cur, 0xFF, sizeof(cur));
    memset(tgt, 0xFF, sizeof(tgt));
    tgt[0] = 0x79;              // px0 7 -> black, px1 9 -> white (stays 15)
    tgt[1] = 0x97;              // px2 9 -> white, px3 7 -> black
    cur[2] = 0x7F;              // px4 gray 7 shown (a GL16 run), target 7: unchanged, stays 7
    tgt[2] = 0x7F;
    cur[3] = 0x00;              // px6, px7 black; target 9 and 8: white
    tgt[3] = 0x98;
    memcpy(g, cur, sizeof(g));
    const epd_rect_t band = {0, 0, W, H};
    epd_wf_commit_bin(cur, tgt, PS, band);
    CHECK(cur[0] == 0x0F);
    CHECK(cur[1] == 0xF0);
    CHECK(cur[2] == 0x7F);
    CHECK(cur[3] == 0xFF);
    CHECK(cur[PS] == 0xFF);
    // GL16 commit keeps the gray levels
    epd_wf_commit(g, tgt, PS, band);
    CHECK(g[0] == 0x79 && g[1] == 0x97 && g[3] == 0x98);
    // outside the band: untouched
    memset(cur, 0x11, sizeof(cur));
    memset(tgt, 0x99, sizeof(tgt));
    epd_wf_commit_bin(cur, tgt, PS, (epd_rect_t){4, 1, 8, 1});
    for (int y = 0; y < H; y++) {
        for (int b = 0; b < PS; b++) {
            const bool in = y == 1 && b >= 2 && b < 6;
            CHECK(cur[y * PS + b] == (in ? 0xFF : 0x11));
        }
    }
}

enum { FW = 1216, FH = 684, FPS = FW / 2 };
static uint8_t f_cur[FPS * FH], f_next[FPS * FH], f_com[FPS * FH];
static uint16_t f_pp[FPS * FH], f_ref[FPS * FH];

static void test_wf_pair_plane_bin_matches_scalar(void)
{
    srand(11);
    for (size_t i = 0; i < sizeof(f_cur); i++) {
        f_cur[i] = (uint8_t)rand();
        // about half the pixels keep their level, so both branches are hit
        f_next[i] = (rand() & 1) ? f_cur[i] : (uint8_t)rand();
        if (rand() % 3 == 0) {
            f_next[i] = (uint8_t)((f_next[i] & 0xF0) | (f_cur[i] & 0x0F));
        }
    }
    const epd_rect_t bands[] = {{0, 0, FW, FH}, {4, 1, 1204, 5}, {8, 2, 20, 3}, {12, 0, 4, 2}, {1212, 3, 4, 4},
                                {28, 5, 1000, 600}, {0, 683, 1216, 1}};
    for (size_t k = 0; k < sizeof(bands) / sizeof(bands[0]); k++) {
        const epd_rect_t b = bands[k];
        memset(f_pp, 0xEE, sizeof(f_pp));
        memset(f_ref, 0xEE, sizeof(f_ref));
        for (int y = b.y; y < b.y + b.h; y++) {
            for (int x = b.x; x < b.x + b.w; x += 2) {
                const size_t o = (size_t)y * FPS + x / 2;
                const int c0 = pxb(f_cur, FPS, x, y), c1 = pxb(f_cur, FPS, x + 1, y);
                const int t0 = bin_px(c0, pxb(f_next, FPS, x, y)), t1 = bin_px(c1, pxb(f_next, FPS, x + 1, y));
                f_ref[o] = (uint16_t)((f_cur[o] << 8) | (t0 << 4) | t1);
            }
        }
        epd_wf_pair_plane_bin(f_cur, f_next, FPS, f_pp, b);
        CHECK(memcmp(f_pp, f_ref, sizeof(f_pp)) == 0);
        // the commit stores the same targets the pair plane drove to
        memcpy(f_com, f_cur, sizeof(f_com));
        epd_wf_commit_bin(f_com, f_next, FPS, b);
        bool same = true;
        for (int y = b.y; y < b.y + b.h && same; y++) {
            for (int x = b.x; x < b.x + b.w; x += 2) {
                const size_t o = (size_t)y * FPS + x / 2;
                same &= f_com[o] == (uint8_t)(f_ref[o] & 0xFF);
            }
        }
        CHECK(same);
        // and says "unchanged" exactly when the pair plane holds no change
        const epd_rect_t box = epd_wf_changed_box(f_pp, FPS, b);
        CHECK(epd_wf_unchanged(f_cur, f_next, FPS, b, true) == (box.w == 0));
    }
}

// A DU run from a gray level (left by a GL16 run) ends at the binary target:
// the table only darkens towards black and only lightens towards white
static void test_wf_du_from_gray_drives_to_endpoint(void)
{
    const epd_wf_kind_t kinds[] = {EPD_WF_DU, EPD_WF_DU_FAST};
    for (size_t k = 0; k < 2; k++) {
        const epd_wf_phases_t *w = epd_wf_pick(&EPD_WF_E0470, kinds[k], 25);
        for (int from = 1; from < 15; from++) {
            const int tos[2] = {bin_px(from, 7), bin_px(from, 9)};
            for (int i = 0; i < 2; i++) {
                const int to = tos[i];
                if (to == from) {
                    continue;
                }
                int toward = 0, away = 0;
                for (int p = 0; p < w->phases; p++) {
                    uint8_t d[EPD_WF_LEVELS][4];
                    epd_wf_phase_data(w, p, EPD_WF_SKIP_SAME_KEEP_BLACK, d);
                    const int a = (d[to][from / 4] >> (6 - 2 * (from % 4))) & 3;
                    toward += a == (to == 0 ? 1 : 2);
                    away += a == (to == 0 ? 2 : 1);
                }
                CHECK(toward > 0);
                CHECK(toward > away);
            }
        }
    }
}

static void test_wf_unchanged_raw_and_bin(void)
{
    enum { W = 32, H = 4, PS = W / 2 };
    uint8_t cur[PS * H], next[PS * H];
    memset(cur, 0xFF, sizeof(cur));
    memcpy(next, cur, sizeof(next));
    const epd_rect_t all = {0, 0, W, H};
    CHECK(epd_wf_unchanged(cur, next, PS, all, false));
    CHECK(epd_wf_unchanged(cur, next, PS, all, true));
    next[PS + 3] = 0xF9;        // anti-aliased 9 over white: no change for DU, a change for GL16
    CHECK(!epd_wf_unchanged(cur, next, PS, all, false));
    CHECK(epd_wf_unchanged(cur, next, PS, all, true));
    next[PS + 3] = 0xF7;        // 7 over white: black for DU
    CHECK(!epd_wf_unchanged(cur, next, PS, all, true));
    CHECK(epd_wf_unchanged(cur, next, PS, (epd_rect_t){8, 0, 8, H}, true));    // outside the rect
    CHECK(epd_wf_unchanged(cur, next, PS, (epd_rect_t){0, 0, 0, 0}, true));     // empty rect
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
    test_wf_transition_is_du();
    test_wf_phase_words_match_bytes();
    test_wf_phase2_matches_two_calls();
    test_wf_phase_same();
    test_wf_pair_plane_kernel_matches();
    test_wf_trans_plane_kernel_matches();
    test_wf_kind_for_modes();
    test_wf_rule_gray_differential();
    test_wf_du_commit_binarizes();
    test_wf_pair_plane_bin_matches_scalar();
    test_wf_du_from_gray_drives_to_endpoint();
    test_wf_unchanged_raw_and_bin();
    TEST_DONE("wf");
}
