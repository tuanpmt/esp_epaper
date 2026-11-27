#include <stdlib.h>
#include <string.h>
#include "check.h"
#include "epd_bus_rec.h"
#include "epd_ssd1677.h"
#include "ssd1677_lut.h"

static epd_bus_rec_t R;
static epd_ssd1677_t *C;
static int done_calls;

static void on_done(void *ctx)
{
    (void)ctx;
    done_calls++;
}

static void setup(bool auto_old)
{
    epd_bus_rec_free(&R);
    epd_bus_rec_init(&R);
    epd_ssd1677_free(C);
    C = NULL;
    done_calls = 0;
    const epd_ssd1677_cfg_t cfg = {.bus = &R.bus, .pan_w = 800, .pan_h = 480, .auto_old_plane = auto_old};
    CHECK(epd_ssd1677_init(&cfg, &C) == ESP_OK);
    // The tests below assume the stock tables, whatever the default picks are
    CHECK(epd_ssd1677_set_partial_timing(C, NULL) == ESP_OK);
    CHECK(epd_ssd1677_set_fast_timing(C, NULL) == ESP_OK);
}

// Index of the n-th record of command cmd (n from 0), -1 when absent
static int nth_cmd(uint8_t cmd, int n)
{
    int at = -1;
    for (int i = 0; i <= n; i++) {
        at = epd_bus_rec_find_cmd(&R, cmd, (size_t)(at + 1));
        if (at < 0) {
            return -1;
        }
    }
    return at;
}

static bool cmd_is(int idx, uint8_t cmd, const uint8_t *data, size_t len)
{
    if (idx < 0 || (size_t)idx >= R.n) {
        return false;
    }
    const epd_rec_t *e = &R.log[idx];
    return e->type == EPD_REC_CMD && e->cmd == cmd && e->len == len && (len == 0 || memcmp(e->bytes, data, len) == 0);
}

static int count_cmd(uint8_t cmd)
{
    int n = 0;
    for (size_t i = 0; i < R.n; i++) {
        n += R.log[i].type == EPD_REC_CMD && R.log[i].cmd == cmd;
    }
    return n;
}

static void test_ssd1677_init_sequence(void)
{
    setup(false);
    CHECK(R.n > 0 && R.log[0].type == EPD_REC_RESET);
    const int swrst = nth_cmd(0x12, 0);
    const int temp = nth_cmd(0x18, 0);
    const int pp = nth_cmd(0x37, 0);
    const int boost = nth_cmd(0x0C, 0);
    const int drv = nth_cmd(0x01, 0);
    const int border = nth_cmd(0x3C, 0);
    const int vcom = nth_cmd(0x2C, 0);
    const int gate = nth_cmd(0x03, 0);
    const int src = nth_cmd(0x04, 0);
    const int lut = nth_cmd(0x32, 0);
    CHECK(swrst > 0 && swrst < temp && temp < pp && pp < boost && boost < drv && drv < border);
    CHECK(border < vcom && vcom < gate && gate < src && src < lut);   // stock loader order: VCOM, gate, source, wave
    CHECK(cmd_is(temp, 0x18, (const uint8_t[]){0x80}, 1));
    CHECK(cmd_is(pp, 0x37, (const uint8_t[]){0, 0, 0, 0, 0, 0x40, 0, 0, 0, 0}, 10));
    CHECK(cmd_is(boost, 0x0C, (const uint8_t[]){0xAE, 0xC7, 0xC3, 0xC0, 0x80}, 5));
    CHECK(cmd_is(drv, 0x01, (const uint8_t[]){0xDF, 0x01, 0x02}, 3));
    CHECK(cmd_is(border, 0x3C, (const uint8_t[]){0x01}, 1));
    CHECK(cmd_is(vcom, 0x2C, (const uint8_t[]){0x50}, 1));          // fast LUT VCOM
    CHECK(R.log[lut].len == 105);
}

static void test_ssd1677_window_math(void)
{
    setup(false);
    epd_bus_rec_clear(&R);
    uint8_t plane[800 / 8 * 480];
    memset(plane, 0xAA, sizeof(plane));
    const epd_rect_t r = {16, 100, 64, 40};
    CHECK(epd_ssd1677_write(C, 0x24, r, plane, 100) == ESP_OK);
    const uint16_t yrev = 480 - 100 - 40;   // 340
    const uint16_t ytop = yrev + 40 - 1;     // 379
    CHECK(cmd_is(nth_cmd(0x11, 0), 0x11, (const uint8_t[]){0x01}, 1));
    CHECK(cmd_is(nth_cmd(0x44, 0), 0x44, (const uint8_t[]){0x10, 0x00, 0x4F, 0x00}, 4));
    CHECK(cmd_is(nth_cmd(0x45, 0), 0x45,
                 (const uint8_t[]){ytop & 0xFF, ytop >> 8, yrev & 0xFF, yrev >> 8}, 4));
    CHECK(cmd_is(nth_cmd(0x4E, 0), 0x4E, (const uint8_t[]){0x10, 0x00}, 2));
    CHECK(cmd_is(nth_cmd(0x4F, 0), 0x4F, (const uint8_t[]){ytop & 0xFF, ytop >> 8}, 2));
    const int ram = nth_cmd(0x24, 0);
    CHECK(ram >= 0);
    size_t bytes = 0;
    for (size_t i = (size_t)ram + 1; i < R.n && R.log[i].type == EPD_REC_DATA; i++) {
        bytes += R.log[i].len;
    }
    CHECK(bytes == 64 * 40 / 8);
    // unaligned x or width is refused
    CHECK(epd_ssd1677_write(C, 0x24, (epd_rect_t){4, 0, 8, 8}, plane, 100) == ESP_ERR_INVALID_ARG);
    CHECK(epd_ssd1677_write(C, 0x24, (epd_rect_t){792, 0, 16, 8}, plane, 100) == ESP_ERR_INVALID_ARG);
}

static void test_ssd1677_partial_writes_old_plane_after(void)
{
    uint8_t plane[800 / 8 * 480] = {0};
    const epd_rect_t r = {0, 0, 32, 16};
    setup(false);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    CHECK(cmd_is(nth_cmd(0x22, 0), 0x22, (const uint8_t[]){0xCF}, 1));
    CHECK(nth_cmd(0x20, 0) > nth_cmd(0x22, 0));
    epd_bus_rec_fire_busy(&R);
    CHECK(done_calls == 1);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_after(C, EPD_M_PARTIAL, r, plane, 100) == ESP_OK);
    CHECK(nth_cmd(0x26, 0) >= 0);

    setup(true);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_after(C, EPD_M_PARTIAL, r, plane, 100) == ESP_OK);
    CHECK(nth_cmd(0x26, 0) < 0);
}

// A transition runs on the partial LUT and keeps the old plane like a partial
static void test_ssd1677_transition_is_partial(void)
{
    uint8_t plane[800 / 8 * 480] = {0};
    setup(false);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_TRANSITION, on_done, NULL) == ESP_OK);
    CHECK(cmd_is(nth_cmd(0x22, 0), 0x22, (const uint8_t[]){0xCF}, 1));
    epd_bus_rec_fire_busy(&R);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_after(C, EPD_M_TRANSITION, (epd_rect_t){0, 0, 32, 16}, plane, 100) == ESP_OK);
    CHECK(nth_cmd(0x26, 0) >= 0);
}

static void test_ssd1677_lut_cache(void)
{
    setup(false);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(count_cmd(0x32) == 1);                         // partial LUT once for both
    CHECK(cmd_is(nth_cmd(0x2C, 0), 0x2C, (const uint8_t[]){0x00}, 1));   // partial VCOM
    CHECK(epd_ssd1677_start(C, EPD_M_FULL_CLEAN, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(count_cmd(0x32) == 2);                         // fast LUT back
    CHECK(cmd_is(nth_cmd(0x22, 2), 0x22, (const uint8_t[]){0xC7}, 1));
    CHECK(done_calls == 3);
}

static void test_ssd1677_gray_mode(void)
{
    setup(false);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_GRAY, on_done, NULL) == ESP_OK);
    CHECK(cmd_is(nth_cmd(0x21, 0), 0x21, (const uint8_t[]){0x00}, 1));
    CHECK(cmd_is(nth_cmd(0x22, 0), 0x22, (const uint8_t[]){0xCC}, 1));
    CHECK(cmd_is(nth_cmd(0x2C, 0), 0x2C, (const uint8_t[]){0x30}, 1));   // 4-gray VCOM
}

// #539: the limit is 3 s of clock time even when every delay_ms(1) sleeps a 10 ms tick
static void test_ssd1677_busy_timeout_real_time(void)
{
    setup(false);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    epd_bus_rec_set_busy(&R, true);
    R.tick_ms = 10;
    const uint32_t t0 = R.clock_ms;
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);
    CHECK(epd_ssd1677_start(C, EPD_M_FULL_CLEAN, on_done, NULL) == ESP_ERR_TIMEOUT);
    const uint32_t took = R.clock_ms - t0;
    CHECK(took >= 3000 && took <= 3000 + 10);   // 3 s +- 1 tick, not 30 s
    R.tick_ms = 0;
}

static void test_ssd1677_busy_timeout(void)
{
    setup(false);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);   // partial LUT loaded
    epd_bus_rec_fire_busy(&R);
    done_calls = 0;
    epd_bus_rec_set_busy(&R, true);     // the panel takes the command and never releases BUSY
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);
    CHECK(done_calls == 0);             // never fires while BUSY stays high
    CHECK(epd_ssd1677_start(C, EPD_M_FULL_CLEAN, on_done, NULL) == ESP_ERR_TIMEOUT);   // LUT load waits on BUSY
    epd_bus_rec_set_busy(&R, false);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_reinit(C) == ESP_OK);
    CHECK(R.n > 0 && R.log[0].type == EPD_REC_RESET);
    CHECK(nth_cmd(0x12, 0) > 0 && nth_cmd(0x32, 0) > 0);   // full init again, LUT reloaded
}

static void test_ssd1677_deep_sleep(void)
{
    setup(false);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_deep_sleep(C) == ESP_OK);
    CHECK(cmd_is(nth_cmd(0x10, 0), 0x10, (const uint8_t[]){0x01}, 1));
}

// Shorter timing: group 0 TP A 13 -> 6, group 1 TP A 4 -> 2, faster frame rate
static epd_ssd1677_timing_t shorter_timing(void)
{
    epd_ssd1677_timing_t t;
    epd_ssd1677_partial_timing(C, &t);
    t.tp[0][0] = 6;
    t.tp[1][0] = 2;
    memset(t.fr, 0x44, sizeof(t.fr));
    return t;
}

static void test_ssd1677_lut_patch_only_timing_bytes(void)
{
    setup(false);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);   // stock partial LUT loaded
    epd_bus_rec_fire_busy(&R);
    const epd_ssd1677_timing_t t = shorter_timing();
    CHECK(epd_ssd1677_set_partial_timing(C, &t) == ESP_OK);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);   // patched table reloads
    epd_bus_rec_fire_busy(&R);
    const int lut = nth_cmd(0x32, 0);
    CHECK(lut >= 0 && R.log[lut].len == SSD1677_LUT_WAVE_BYTES);
    if (lut >= 0 && R.log[lut].len == SSD1677_LUT_WAVE_BYTES) {
        const uint8_t *w = R.log[lut].bytes;
        CHECK(memcmp(w, ssd1677_lut_partial, 50) == 0);   // VS phase rows never change
        CHECK(w[50] == 6 && w[55] == 2);                  // TP A of groups 0 and 1
        for (int i = 100; i < 105; i++) {
            CHECK(w[i] == 0x44);                          // frame rate
        }
        int diff = 0;
        for (int i = 0; i < SSD1677_LUT_WAVE_BYTES; i++) {
            diff += w[i] != ssd1677_lut_partial[i];
        }
        CHECK(diff == 7);                                 // nothing else moved
    }
    // voltages keep the stock values
    CHECK(cmd_is(nth_cmd(0x03, 0), 0x03, &ssd1677_lut_partial[105], 1));
    CHECK(cmd_is(nth_cmd(0x04, 0), 0x04, &ssd1677_lut_partial[106], 3));
    CHECK(cmd_is(nth_cmd(0x2C, 0), 0x2C, &ssd1677_lut_partial[109], 1));
    // the stock table itself is untouched
    CHECK(ssd1677_lut_partial[50] == 0x0D && ssd1677_lut_partial[55] == 0x04 && ssd1677_lut_partial[100] == 0x22);

    // the safety boundary: timing may only get shorter than stock
    epd_ssd1677_timing_t longer = t;
    longer.tp[0][0] = 0x0E;
    CHECK(epd_ssd1677_set_partial_timing(C, &longer) == ESP_ERR_INVALID_ARG);
    longer = t;
    longer.tp[2][1] = 1;                                  // a stock-empty phase
    CHECK(epd_ssd1677_set_partial_timing(C, &longer) == ESP_ERR_INVALID_ARG);
    longer = t;
    longer.rp[0] = 1;
    CHECK(epd_ssd1677_set_partial_timing(C, &longer) == ESP_ERR_INVALID_ARG);

    // NULL restores the stock table, loaded again at the next partial
    CHECK(epd_ssd1677_set_partial_timing(C, NULL) == ESP_OK);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    const int stock = nth_cmd(0x32, 0);
    CHECK(cmd_is(stock, 0x32, ssd1677_lut_partial, SSD1677_LUT_WAVE_BYTES));
}

static void test_ssd1677_partial_timing_roundtrip(void)
{
    setup(false);
    epd_ssd1677_timing_t t;
    epd_ssd1677_partial_timing(C, &t);
    CHECK(t.tp[0][0] == 0x0D && t.tp[1][0] == 0x04 && t.fr[0] == 0x22 && t.fr[4] == 0x22);   // stock
    CHECK(t.tp[0][1] == 0 && t.rp[0] == 0);
    const epd_ssd1677_timing_t want = shorter_timing();
    CHECK(epd_ssd1677_set_partial_timing(C, &want) == ESP_OK);
    epd_ssd1677_partial_timing(C, &t);
    CHECK(memcmp(&t, &want, sizeof(t)) == 0);
    // a refused timing leaves the current one in place
    epd_ssd1677_timing_t bad = want;
    bad.tp[9][3] = 0xFF;
    CHECK(epd_ssd1677_set_partial_timing(C, &bad) == ESP_ERR_INVALID_ARG);
    epd_ssd1677_partial_timing(C, &t);
    CHECK(memcmp(&t, &want, sizeof(t)) == 0);
    // reinit after a BUSY timeout keeps the tuned timing
    CHECK(epd_ssd1677_reinit(C) == ESP_OK);
    epd_ssd1677_partial_timing(C, &t);
    CHECK(memcmp(&t, &want, sizeof(t)) == 0);
}

// BUSY rises a little after 0x20: arming the idle interrupt before that would
// report the refresh done at once. start() polls (1 ms steps) until BUSY rises.
static void test_ssd1677_arm_waits_for_busy_high(void)
{
    setup(true);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);   // LUT loaded
    epd_bus_rec_fire_busy(&R);
    epd_bus_rec_set_busy(&R, true);
    R.low_reads = 3;
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    const int act = epd_bus_rec_find_cmd(&R, 0x20, 0);
    CHECK(act >= 0);
    size_t i = (size_t)act + 1;
    int delays = 0;
    while (i < R.n && R.log[i].type == EPD_REC_DELAY) {
        delays++;
        i++;
    }
    CHECK(delays == 3);
    CHECK(i < R.n && R.log[i].type == EPD_REC_ARM);
}

// Index of the last 0x32 (wave table) record, -1 when none
static int last_lut(void)
{
    int at = -1;
    for (size_t i = 0; i < R.n; i++) {
        if (R.log[i].type == EPD_REC_CMD && R.log[i].cmd == 0x32) {
            at = (int)i;
        }
    }
    return at;
}

static bool lut_bytes_are(int idx, const uint8_t *table)
{
    return cmd_is(idx, 0x32, table, SSD1677_LUT_WAVE_BYTES);
}

// FAST (interaction) runs on its own table; PARTIAL and TRANSITION keep the
// partial table, FULL_CLEAN the full one
static void test_ssd1677_fast_lut_used_for_fast_only(void)
{
    setup(true);
    epd_ssd1677_timing_t t;
    epd_ssd1677_fast_timing(C, &t);
    t.tp[0][0] = 6;
    memset(t.fr, 0x44, sizeof(t.fr));
    CHECK(epd_ssd1677_set_fast_timing(C, &t) == ESP_OK);
    epd_ssd1677_timing_t p;
    epd_ssd1677_partial_timing(C, &p);
    CHECK(p.tp[0][0] == 0x0D && p.fr[0] == 0x22);           // the partial table is untouched

    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(lut_bytes_are(last_lut(), ssd1677_lut_partial));

    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    const int fast = last_lut();
    CHECK(fast >= 0 && R.log[fast].len == SSD1677_LUT_WAVE_BYTES);
    if (fast >= 0 && R.log[fast].len == SSD1677_LUT_WAVE_BYTES) {
        CHECK(R.log[fast].bytes[50] == 6 && R.log[fast].bytes[100] == 0x44);
    }
    CHECK(cmd_is(nth_cmd(0x22, 0), 0x22, (const uint8_t[]){0xCF}, 1));   // still a partial update
    CHECK(count_cmd(0x3C) == 0);                                          // border already set for partials

    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);    // cached
    epd_bus_rec_fire_busy(&R);
    CHECK(count_cmd(0x32) == 0);

    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_TRANSITION, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(lut_bytes_are(last_lut(), ssd1677_lut_partial));

    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_FULL_CLEAN, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(lut_bytes_are(last_lut(), ssd1677_lut_fast));
}

// Same tables (the default): switching between PARTIAL and FAST never reloads
static void test_ssd1677_same_tables_no_reload(void)
{
    setup(true);
    epd_bus_rec_clear(&R);
    for (int i = 0; i < 3; i++) {
        CHECK(epd_ssd1677_start(C, i & 1 ? EPD_M_FAST : EPD_M_TRANSITION, on_done, NULL) == ESP_OK);
        epd_bus_rec_fire_busy(&R);
    }
    CHECK(count_cmd(0x32) == 1);
    // a fast table changed while it is the loaded one is loaded again
    epd_ssd1677_timing_t t;
    epd_ssd1677_fast_timing(C, &t);
    memset(t.fr, 0x55, sizeof(t.fr));
    CHECK(epd_ssd1677_set_fast_timing(C, &t) == ESP_OK);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(count_cmd(0x32) == 1);
    // and the partial table, unchanged but no longer loaded, comes back
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(lut_bytes_are(last_lut(), ssd1677_lut_partial));
}

// Same boundary as the partial table: only timing bytes move, only shorter
static void test_ssd1677_fast_patch_only_timing_bytes(void)
{
    setup(true);
    epd_ssd1677_timing_t t;
    epd_ssd1677_fast_timing(C, &t);
    CHECK(t.tp[0][0] == 0x0D && t.tp[1][0] == 0x04 && t.fr[0] == 0x22);   // stock partial timing
    t.tp[0][0] = 6;
    t.tp[1][0] = 2;
    memset(t.fr, 0x44, sizeof(t.fr));
    CHECK(epd_ssd1677_set_fast_timing(C, &t) == ESP_OK);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_FAST, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    const int lut = last_lut();
    CHECK(lut >= 0 && R.log[lut].len == SSD1677_LUT_WAVE_BYTES);
    if (lut >= 0 && R.log[lut].len == SSD1677_LUT_WAVE_BYTES) {
        int diff = 0;
        for (int i = 0; i < SSD1677_LUT_WAVE_BYTES; i++) {
            diff += R.log[lut].bytes[i] != ssd1677_lut_partial[i];
        }
        CHECK(diff == 7);
        CHECK(memcmp(R.log[lut].bytes, ssd1677_lut_partial, 50) == 0);
    }
    // bytes 105..109 (gate, source, VCOM) stay stock
    CHECK(cmd_is(nth_cmd(0x03, 0), 0x03, &ssd1677_lut_partial[105], 1));
    CHECK(cmd_is(nth_cmd(0x04, 0), 0x04, &ssd1677_lut_partial[106], 3));
    CHECK(cmd_is(nth_cmd(0x2C, 0), 0x2C, &ssd1677_lut_partial[109], 1));

    epd_ssd1677_timing_t bad = t;
    bad.tp[0][0] = 0x0E;
    CHECK(epd_ssd1677_set_fast_timing(C, &bad) == ESP_ERR_INVALID_ARG);
    bad = t;
    bad.rp[3] = 1;
    CHECK(epd_ssd1677_set_fast_timing(C, &bad) == ESP_ERR_INVALID_ARG);
    epd_ssd1677_timing_t now;
    epd_ssd1677_fast_timing(C, &now);
    CHECK(memcmp(&now, &t, sizeof(t)) == 0);              // refused: unchanged
    CHECK(epd_ssd1677_reinit(C) == ESP_OK);
    epd_ssd1677_fast_timing(C, &now);
    CHECK(memcmp(&now, &t, sizeof(t)) == 0);              // kept across reinit
    CHECK(epd_ssd1677_set_fast_timing(C, NULL) == ESP_OK);   // back to stock
    epd_ssd1677_fast_timing(C, &now);
    CHECK(now.tp[0][0] == 0x0D && now.fr[4] == 0x22);
}

// The candidates of the LUT sweep (plan 9 Task 8), numbered as on the panel
static void test_ssd1677_lut_candidates(void)
{
    setup(true);
    epd_ssd1677_timing_t stock, c;
    epd_ssd1677_partial_timing(C, &stock);
    CHECK(epd_ssd1677_lut_candidate(1, &c) == ESP_OK && memcmp(&c, &stock, sizeof(c)) == 0);
    const uint8_t fr[] = {0, 0x22, 0x44, 0x44, 0x55, 0x55};
    const uint8_t tp0[] = {0, 0x0D, 0x0D, 6, 0x0D, 6};
    for (uint8_t id = 1; id <= EPD_SSD1677_LUT_CANDIDATES; id++) {
        CHECK(epd_ssd1677_lut_candidate(id, &c) == ESP_OK);
        CHECK(c.fr[0] == fr[id] && c.fr[4] == fr[id] && c.tp[0][0] == tp0[id]);
        epd_ssd1677_timing_t rest = c, ref = stock;
        rest.tp[0][0] = ref.tp[0][0] = 0;
        memset(rest.fr, 0, sizeof(rest.fr));
        memset(ref.fr, 0, sizeof(ref.fr));
        CHECK(memcmp(&rest, &ref, sizeof(rest)) == 0);    // nothing else moves
        CHECK(epd_ssd1677_set_partial_timing(C, &c) == ESP_OK);
        CHECK(epd_ssd1677_set_fast_timing(C, &c) == ESP_OK);
    }
    CHECK(epd_ssd1677_lut_candidate(0, &c) == ESP_ERR_INVALID_ARG);
    CHECK(epd_ssd1677_lut_candidate(EPD_SSD1677_LUT_CANDIDATES + 1, &c) == ESP_ERR_INVALID_ARG);
}

// init applies the picked candidates of ssd1677_lut.c to both tables
static void test_ssd1677_init_applies_picks(void)
{
    epd_bus_rec_free(&R);
    epd_bus_rec_init(&R);
    epd_ssd1677_free(C);
    C = NULL;
    const epd_ssd1677_cfg_t cfg = {.bus = &R.bus, .pan_w = 800, .pan_h = 480, .auto_old_plane = true};
    CHECK(epd_ssd1677_init(&cfg, &C) == ESP_OK);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_PAGE) == ssd1677_lut_pick_partial);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_SCREEN) == ssd1677_lut_pick_partial);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_BUTTON) == ssd1677_lut_pick_fast);
    uint8_t want[EPD_SSD1677_LUT_SIZE];
    CHECK(epd_ssd1677_table(ssd1677_lut_pick_fast, want) == ESP_OK);
    epd_ssd1677_timing_t got;
    epd_ssd1677_fast_timing(C, &got);
    CHECK(got.tp[0][0] == want[50] && got.fr[0] == want[100]);
}

// Candidate tables of plan 9 Task 12: all inside the safety envelope, 1..5
// equal to the timing candidates, voltages never touched
static void test_ssd1677_tables_safe(void)
{
    uint8_t t[EPD_SSD1677_LUT_SIZE];
    for (uint8_t id = 1; id <= EPD_SSD1677_LUT_TABLES; id++) {
        CHECK(epd_ssd1677_table(id, t) == ESP_OK);
        CHECK(epd_ssd1677_table_name(id) != NULL);
        CHECK(memcmp(&t[105], &ssd1677_lut_partial[105], 7) == 0);
        CHECK(memcmp(&t[40], &ssd1677_lut_partial[40], 10) == 0);   // VCOM row
        int frames = 0;
        for (int i = 50; i < 100; i++) {
            frames += i % 5 == 4 ? 0 : t[i];
        }
        CHECK(frames <= 17);
        if (id <= EPD_SSD1677_LUT_CANDIDATES) {
            epd_ssd1677_timing_t c;
            CHECK(epd_ssd1677_lut_candidate(id, &c) == ESP_OK);
            CHECK(memcmp(t, ssd1677_lut_partial, 50) == 0);
            CHECK(t[50] == c.tp[0][0] && t[100] == c.fr[0] && t[104] == c.fr[4]);
        }
    }
    CHECK(epd_ssd1677_table(0, t) == ESP_ERR_INVALID_ARG);
    CHECK(epd_ssd1677_table(EPD_SSD1677_LUT_TABLES + 1, t) == ESP_ERR_INVALID_ARG);
    // deepen: both directions repeated in group 1 (DC balanced)
    CHECK(epd_ssd1677_table(8, t) == ESP_OK && t[11] == 0x80 && t[21] == 0x40 && t[31] == 0);
    // the envelope refuses voltages, longer phases, slower frames, late groups
    uint8_t bad[EPD_SSD1677_LUT_SIZE];
    memcpy(bad, ssd1677_lut_partial, sizeof(bad));
    CHECK(epd_ssd1677_table_check(bad) == ESP_OK);
    bad[107] ^= 1;
    CHECK(epd_ssd1677_table_check(bad) == ESP_ERR_INVALID_ARG);
    memcpy(bad, ssd1677_lut_partial, sizeof(bad));
    bad[50] = 0x0E;
    CHECK(epd_ssd1677_table_check(bad) == ESP_ERR_INVALID_ARG);
    memcpy(bad, ssd1677_lut_partial, sizeof(bad));
    bad[100] = 0x11;
    CHECK(epd_ssd1677_table_check(bad) == ESP_ERR_INVALID_ARG);
    memcpy(bad, ssd1677_lut_partial, sizeof(bad));
    bad[12] = 0x40;
    CHECK(epd_ssd1677_table_check(bad) == ESP_ERR_INVALID_ARG);
}

// Each role loads its own table; switching at run time reloads the LUT
static void test_ssd1677_tables_per_role(void)
{
    epd_bus_rec_free(&R);
    epd_bus_rec_init(&R);
    epd_ssd1677_free(C);
    C = NULL;
    const epd_ssd1677_cfg_t cfg = {.bus = &R.bus, .pan_w = 800, .pan_h = 480, .auto_old_plane = true};
    CHECK(epd_ssd1677_init(&cfg, &C) == ESP_OK);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_PAGE) == ssd1677_lut_pick_partial);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_SCREEN) == ssd1677_lut_pick_partial);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_BUTTON) == ssd1677_lut_pick_fast);
    uint8_t want[EPD_SSD1677_LUT_SIZE];
    CHECK(epd_ssd1677_use_table(C, EPD_SSD1677_ROLE_SCREEN, 11) == ESP_OK);
    CHECK(epd_ssd1677_table(11, want) == ESP_OK);
    epd_bus_rec_clear(&R);
    CHECK(epd_ssd1677_start(C, EPD_M_TRANSITION, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(lut_bytes_are(last_lut(), want));
    CHECK(epd_ssd1677_table(ssd1677_lut_pick_partial, want) == ESP_OK);
    CHECK(epd_ssd1677_start(C, EPD_M_PARTIAL, on_done, NULL) == ESP_OK);
    epd_bus_rec_fire_busy(&R);
    CHECK(lut_bytes_are(last_lut(), want));
    CHECK(epd_ssd1677_use_table(C, EPD_SSD1677_ROLE_PAGE, 0) == ESP_ERR_INVALID_ARG);
    CHECK(epd_ssd1677_use_table(C, EPD_SSD1677_ROLE_COUNT, 1) == ESP_ERR_INVALID_ARG);
    CHECK(epd_ssd1677_set_partial_timing(C, NULL) == ESP_OK);
    CHECK(epd_ssd1677_table_in_use(C, EPD_SSD1677_ROLE_PAGE) == 0);
}

int main(void)
{
    test_ssd1677_tables_safe();
    test_ssd1677_tables_per_role();
    test_ssd1677_arm_waits_for_busy_high();
    test_ssd1677_init_sequence();
    test_ssd1677_window_math();
    test_ssd1677_partial_writes_old_plane_after();
    test_ssd1677_transition_is_partial();
    test_ssd1677_lut_cache();
    test_ssd1677_gray_mode();
    test_ssd1677_busy_timeout();
    test_ssd1677_busy_timeout_real_time();
    test_ssd1677_deep_sleep();
    test_ssd1677_lut_patch_only_timing_bytes();
    test_ssd1677_partial_timing_roundtrip();
    test_ssd1677_fast_lut_used_for_fast_only();
    test_ssd1677_same_tables_no_reload();
    test_ssd1677_fast_patch_only_timing_bytes();
    test_ssd1677_lut_candidates();
    test_ssd1677_init_applies_picks();
    epd_ssd1677_free(C);
    epd_bus_rec_free(&R);
    TEST_DONE("ssd1677");
}
