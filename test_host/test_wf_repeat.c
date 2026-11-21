// Repeated differential refreshes through the real LUT + pair-plane kernel
// path (epd_wf_job_*): pixels that keep their level get no drive, the
// previous-frame plane ends equal to the last target, and a full GC16 drives
// exactly the vendor table.
#include <stdlib.h>
#include <string.h>
#include "check.h"
#include "epd_wf.h"

enum { W = 256, H = 32, PS = W / 2, OS = W / 4, MAXPH = 64 };

static _Alignas(8) uint8_t s_cur[PS * H], s_tgt[PS * H], s_img[PS * H], s_img2[PS * H];
static _Alignas(8) uint16_t s_pp[PS * H];
static _Alignas(8) uint8_t s_buf[2][OS * H];
static _Alignas(8) uint8_t s_lut[EPD_WF_PAIR_LUT_SIZE];
static uint8_t s_codes[MAXPH][W * H];   // bus code per phase and pixel of the last run

static const epd_rect_t ALL = {0, 0, W, H};

static int px(const uint8_t *p, int x, int y)
{
    const uint8_t b = p[y * PS + x / 2];
    return x & 1 ? b & 15 : b >> 4;
}

static void set_px(uint8_t *p, int x, int y, int v)
{
    uint8_t *b = &p[y * PS + x / 2];
    *b = (uint8_t)(x & 1 ? (*b & 0xF0) | v : (*b & 0x0F) | (v << 4));
}

// A launcher-like frame: white page, black glyph blocks with anti-aliased
// edges of every level, a light gray card
static void make_image(uint8_t *p)
{
    memset(p, 0xFF, PS * H);
    for (int y = 4; y < 28; y++) {
        for (int x = 8; x < 120; x++) {
            set_px(p, x, y, 13);    // card
        }
    }
    for (int g = 0; g < 8; g++) {           // glyphs: black core, edge ramp 1..14
        const int x0 = 16 + g * 28, y0 = 6 + (g & 3);
        for (int y = y0; y < y0 + 18; y++) {
            for (int x = x0; x < x0 + 20; x++) {
                const int d = (x - x0 < 6) ? x - x0 : (x0 + 19 - x < 6 ? x0 + 19 - x : 6);
                set_px(p, x, y, d >= 6 ? 0 : (d * 3 + g) % 15);
            }
        }
    }
}

typedef struct {
    uint64_t idle_seen;     // non-zero codes on unchanged pixels of levels 1..15
    uint64_t hold_seen;     // non-zero codes on unchanged black pixels
    uint64_t changed_wrong; // codes on changed pixels that differ from the rule's table
    int phases;
} run_t;

// One refresh as the Plus backend runs it: pair plane cur -> target, job
// phases (reused phases scan the previous buffer again), commit on success.
static run_t refresh(const epd_wf_phases_t *w, epd_wf_rule_t rule, uint8_t *cur, const uint8_t *tgt, epd_rect_t band,
                     epd_wf_job_t *j)
{
    run_t r = {.phases = w->phases};
    epd_wf_pair_plane(cur, tgt, PS, s_pp, band);
    memset(s_buf, 0, sizeof(s_buf));
    epd_wf_job_begin(j, w, rule, s_lut, s_pp, PS, band, epd_wf_phase_pp);
    int bi = 0;
    epd_wf_job_phase(j, 0, s_buf[0], OS);
    for (int k = 0; k < w->phases; k++) {
        if (k > 0 && epd_wf_job_phase(j, k, s_buf[bi ^ 1], OS)) {
            bi ^= 1;
        }
        uint8_t d[EPD_WF_LEVELS][4];
        epd_wf_phase_data(w, k, rule, d);
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                const int code = (s_buf[bi][y * OS + x / 4] >> (2 * (x % 4))) & 3;
                s_codes[k][y * W + x] = (uint8_t)code;
                const int f = px(cur, x, y), t = px(tgt, x, y);
                const bool in = x >= band.x && x < band.x + band.w && y >= band.y && y < band.y + band.h;
                if (!in) {
                    r.changed_wrong += code != 0;
                } else if (f == t) {
                    if (code) {
                        if (f) {
                            r.idle_seen++;
                        } else {
                            r.hold_seen++;
                        }
                    }
                } else {
                    r.changed_wrong += code != ((d[t][f / 4] >> (6 - 2 * (f % 4))) & 3);
                }
            }
        }
    }
    epd_wf_commit(cur, tgt, PS, band);
    return r;
}

static const epd_wf_phases_t *table(epd_wf_kind_t k)
{
    return epd_wf_pick(&EPD_WF_E0470, k, EPD_WF_TEMP_DEFAULT);
}

static int raw(const epd_wf_phases_t *w, int p, int from, int to)
{
    return epd_wf_action(w, p, from, to);
}

static int data_act(const uint8_t d[EPD_WF_LEVELS][4], int from, int to)
{
    return (d[to][from / 4] >> (6 - 2 * (from % 4))) & 3;
}

static void test_rule_for(void)
{
    CHECK(epd_wf_rule_for(EPD_M_FAST, EPD_WF_DU) == EPD_WF_SKIP_SAME_KEEP_BLACK);
    CHECK(epd_wf_rule_for(EPD_M_PARTIAL, EPD_WF_DU) == EPD_WF_SKIP_SAME_KEEP_BLACK);
    CHECK(epd_wf_rule_for(EPD_M_PARTIAL, EPD_WF_DU_FAST) == EPD_WF_SKIP_SAME_KEEP_BLACK);
    CHECK(epd_wf_rule_for(EPD_M_FAST, EPD_WF_GC16) == EPD_WF_DRIVE_ALL);       // cold: full GC16
    CHECK(epd_wf_rule_for(EPD_M_FULL_CLEAN, EPD_WF_GC16) == EPD_WF_DRIVE_ALL);
    CHECK(epd_wf_rule_for(EPD_M_GRAY, EPD_WF_GL16) == EPD_WF_DRIVE_ALL);       // vendor: plain GL16 is a full draw
    CHECK(epd_wf_rule_for(EPD_M_PARTIAL, EPD_WF_GL16) == EPD_WF_SKIP_SAME_GL16);
}

// Every phase of every E0470 table, each rule against the raw table
static void test_phase_data_rules(void)
{
    for (int k = 0; k < EPD_WF_COUNT; k++) {
        const epd_wf_phases_t *w = table((epd_wf_kind_t)k);
        int darken[MAXPH], nd = 0;      // phases where white -> black darkens
        for (int p = 0; p < w->phases; p++) {
            if (raw(w, p, 15, 0) == 1) {
                darken[nd++] = p;
            }
        }
        for (int p = 0; p < w->phases; p++) {
            uint8_t a[EPD_WF_LEVELS][4], s[EPD_WF_LEVELS][4], b[EPD_WF_LEVELS][4], g[EPD_WF_LEVELS][4];
            epd_wf_phase_data(w, p, EPD_WF_DRIVE_ALL, a);
            epd_wf_phase_data(w, p, EPD_WF_SKIP_SAME, s);
            epd_wf_phase_data(w, p, EPD_WF_SKIP_SAME_KEEP_BLACK, b);
            epd_wf_phase_data(w, p, EPD_WF_SKIP_SAME_GL16, g);
            bool last4 = false;
            for (int i = nd - 4 < 0 ? 0 : nd - 4; i < nd; i++) {
                last4 |= darken[i] == p;
            }
            for (int f = 0; f < 16; f++) {
                for (int t = 0; t < 16; t++) {
                    const int r = raw(w, p, f, t);
                    CHECK(data_act(a, f, t) == r);
                    CHECK(data_act(s, f, t) == (f == t ? 0 : r));
                    CHECK(data_act(b, f, t) == (f == t && f ? 0 : r));
                    int eg = f == t ? 0 : r;
                    if (t == 0 && f != 0) {
                        eg = r == 1 ? 1 : 0;
                    }
                    if (t == 0 && f == 0) {
                        eg = last4 ? 1 : 0;
                    }
                    CHECK(data_act(g, f, t) == eg);
                }
            }
        }
    }
}

// The device report: DU every ~5 s over the launcher band with an unchanged
// image. Before the fix every run pushed the anti-aliased edges and the card
// (levels 1..14) again; now only black keeps the vendor's DU black hold.
static void test_du_repeat_same_image(void)
{
    make_image(s_img);
    memset(s_cur, 0xFF, sizeof(s_cur));
    const epd_wf_phases_t *w = table(epd_wf_kind_for(EPD_M_FAST, 25, false));
    const epd_wf_rule_t rule = epd_wf_rule_for(EPD_M_FAST, EPD_WF_DU);
    epd_wf_job_t j;
    const epd_rect_t band = {4, 0, W - 8, H};
    for (int n = 0; n < 10; n++) {
        const run_t r = refresh(w, rule, s_cur, s_img, band, &j);
        CHECK(r.idle_seen == 0);
        CHECK(r.changed_wrong == 0);
        CHECK(j.idle_drive == r.idle_seen);
        CHECK(j.black_hold == r.hold_seen);
        CHECK(memcmp(s_cur, s_img, sizeof(s_cur)) == 0);
        if (n > 0) {
            CHECK(r.hold_seen > 0);     // black text in the band: DU's 0 -> 0 hold, as the vendor DU
            for (int k = 0; k < w->phases; k++) {
                for (int i = 0; i < W * H; i++) {
                    const int lvl = px(s_img, i % W, i / W);
                    if (lvl) {
                        CHECK(s_codes[k][i] == 0);
                    }
                }
            }
        }
    }
    // cur equals the last target everywhere the band covered; outside it, the boot white
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const bool in = x >= band.x && x < band.x + band.w;
            CHECK(px(s_cur, x, y) == (in ? px(s_img, x, y) : 15));
        }
    }
}

// The counter is real: the old path (raw table on a DU partial) drives the
// unchanged grays, and job.idle_drive equals the codes on the bus.
static void test_idle_counter_sees_raw_table(void)
{
    make_image(s_img);
    memcpy(s_cur, s_img, sizeof(s_cur));
    const epd_wf_phases_t *w = table(EPD_WF_DU);
    epd_wf_job_t j;
    const run_t r = refresh(w, EPD_WF_DRIVE_ALL, s_cur, s_img, ALL, &j);
    CHECK(r.idle_seen > 0);
    CHECK(j.idle_drive == r.idle_seen);
    CHECK(j.black_hold == r.hold_seen);
}

// A small change (a clock digit) after the full image: only the changed
// pixels move; the rest of the band gets nothing but the black hold
static void test_du_small_change(void)
{
    make_image(s_img);
    memcpy(s_img2, s_img, sizeof(s_img2));
    for (int y = 10; y < 20; y++) {
        for (int x = 200; x < 212; x++) {
            set_px(s_img2, x, y, (x + y) & 1 ? 0 : 15);
        }
    }
    memcpy(s_cur, s_img, sizeof(s_cur));
    const epd_wf_phases_t *w = table(EPD_WF_DU);
    epd_wf_job_t j;
    const run_t r = refresh(w, EPD_WF_SKIP_SAME_KEEP_BLACK, s_cur, s_img2, ALL, &j);
    CHECK(r.idle_seen == 0);
    CHECK(r.changed_wrong == 0);
    CHECK(j.idle_drive == 0);
    CHECK(memcmp(s_cur, s_img2, sizeof(s_cur)) == 0);
}

static void test_gl16_repeat_same_image(void)
{
    make_image(s_img);
    memcpy(s_cur, s_img, sizeof(s_cur));
    const epd_wf_phases_t *w = table(EPD_WF_GL16);
    epd_wf_job_t j;
    for (int n = 0; n < 3; n++) {
        const run_t r = refresh(w, EPD_WF_SKIP_SAME_GL16, s_cur, s_img, ALL, &j);
        CHECK(r.idle_seen == 0);
        CHECK(j.idle_drive == 0);
        CHECK(j.black_hold == r.hold_seen);
        for (int i = 0; i < W * H; i++) {      // black hold: at most 4 phases per pixel
            int c = 0;
            for (int k = 0; k < w->phases; k++) {
                c += s_codes[k][i] != 0;
            }
            CHECK(c <= 4);
        }
    }
}

// Full clean to white from every level: each pixel gets exactly the vendor
// table's GC16 sequence for from -> 15, white -> white included (the clean
// conditions unchanged pixels too), and the run ends on the table's last codes.
static void test_gc16_white_from_any(void)
{
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            set_px(s_cur, x, y, (x / 16) & 15);
        }
    }
    memset(s_tgt, 0xFF, sizeof(s_tgt));
    const epd_wf_kind_t kind = epd_wf_kind_for(EPD_M_FULL_CLEAN, 25, false);
    const epd_wf_phases_t *w = table(kind);
    CHECK(kind == EPD_WF_GC16 && w->phases == 48);
    const epd_wf_rule_t rule = epd_wf_rule_for(EPD_M_FULL_CLEAN, kind);
    CHECK(rule == EPD_WF_DRIVE_ALL);
    uint8_t from[W * H];
    for (int i = 0; i < W * H; i++) {
        from[i] = (uint8_t)px(s_cur, i % W, i / W);
    }
    epd_wf_job_t j;
    refresh(w, rule, s_cur, s_tgt, ALL, &j);
    uint64_t wrong = 0;
    for (int i = 0; i < W * H; i++) {
        for (int k = 0; k < w->phases; k++) {
            wrong += s_codes[k][i] != raw(w, k, from[i], 15);
        }
    }
    CHECK(wrong == 0);
    int ww = 0;
    for (int k = 0; k < w->phases; k++) {
        ww += raw(w, k, 15, 15) != 0;
    }
    CHECK(ww > 0);      // the vendor GC16 drives white -> white
    for (int f = 0; f < 16; f++) {
        CHECK(s_codes[w->phases - 1][f * 16] == raw(w, w->phases - 1, f, 15));
    }
    CHECK(memcmp(s_cur, s_tgt, sizeof(s_cur)) == 0);
}

static void test_hist_and_box(void)
{
    memset(s_cur, 0xFF, sizeof(s_cur));
    memcpy(s_tgt, s_cur, sizeof(s_tgt));
    set_px(s_cur, 10, 3, 0);    // unchanged black
    set_px(s_tgt, 10, 3, 0);
    set_px(s_tgt, 37, 5, 0);    // changed
    set_px(s_tgt, 101, 20, 7);  // changed
    epd_wf_pair_plane(s_cur, s_tgt, PS, s_pp, ALL);
    uint32_t h[EPD_WF_LEVELS];
    epd_wf_same_hist(s_pp, PS, ALL, h);
    CHECK(h[0] == 1);
    CHECK(h[15] == (uint32_t)(W * H - 3));
    CHECK(h[7] == 0);
    const epd_rect_t b = epd_wf_changed_box(s_pp, PS, ALL);
    CHECK(b.x == 37 && b.y == 5 && b.w == 101 - 37 + 1 && b.h == 20 - 5 + 1);
    const epd_rect_t in = {40, 0, 40, H};      // no change inside
    CHECK(epd_wf_changed_box(s_pp, PS, in).w == 0);
    epd_wf_same_hist(s_pp, PS, in, h);
    CHECK(h[15] == 40u * H);
}

int main(void)
{
    test_rule_for();
    test_phase_data_rules();
    test_du_repeat_same_image();
    test_idle_counter_sees_raw_table();
    test_du_small_change();
    test_gl16_repeat_same_image();
    test_gc16_white_from_any();
    test_hist_and_box();
    TEST_DONE("wf_repeat");
}
