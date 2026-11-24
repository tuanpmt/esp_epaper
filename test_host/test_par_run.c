// Refresh run loop: order, pacing and every abort path.
#include <string.h>
#include "check.h"
#include "epd_par_run.h"

typedef struct {
    char log[4096];
    uint64_t now;
    uint32_t frame_us, compute_us;
    int pg_fail_at;     // pg_ok false at this phase check (0-based), -1 never
    int scan_fail_at;   // scan_wait fails for this phase, -1 never
    bool hv_fail;
    uint32_t hv_on_us;      // time hv_on takes (wake, I2C, PG wait)
    int pg_checks, scans, hv_offs, hv_ons;
    int late_at;            // compute of this phase stalls 20 ms
    int stall_at;           // compute of this phase stalls stall_us
    uint32_t stall_us;
    unsigned reuse_mask;    // compute(phase) reports "same as the previous phase" for these phases
    const uint8_t *scanned[64];
    uint64_t scan_t[64];
    uint8_t *computed[64];
} fk_t;

static void lg(fk_t *f, const char *s)
{
    strncat(f->log, s, sizeof(f->log) - strlen(f->log) - 1);
}

static esp_err_t f_hv_on(void *c) { fk_t *f = c; f->hv_ons++; f->now += f->hv_on_us; lg(f, "ON "); return f->hv_fail ? ESP_FAIL : ESP_OK; }
static esp_err_t f_hv_off(void *c) { fk_t *f = c; f->hv_offs++; lg(f, "OFF "); return ESP_OK; }
static bool f_pg(void *c)
{
    fk_t *f = c;
    const bool ok = f->pg_checks != f->pg_fail_at;
    f->pg_checks++;
    lg(f, ok ? "P " : "p ");
    return ok;
}
static bool f_compute(void *c, int phase, uint8_t *buf)
{
    fk_t *f = c;
    char s[16];
    if (phase < 32 && (f->reuse_mask & (1u << phase))) {
        snprintf(s, sizeof(s), "R%d ", phase);
        lg(f, s);
        return false;
    }
    snprintf(s, sizeof(s), "C%d ", phase);
    lg(f, s);
    f->computed[phase] = buf;
    f->now += f->compute_us + (phase == f->late_at ? 20000u : 0u) + (phase == f->stall_at ? f->stall_us : 0u);
    return true;
}
static esp_err_t f_scan_start(void *c, const uint8_t *buf)
{
    fk_t *f = c;
    f->scanned[f->scans] = buf;
    f->scan_t[f->scans] = f->now;
    char s[16];
    snprintf(s, sizeof(s), "S%d ", f->scans);
    lg(f, s);
    return ESP_OK;
}
static esp_err_t f_scan_wait(void *c)
{
    fk_t *f = c;
    const int k = f->scans++;
    const uint64_t end = f->scan_t[k] + f->frame_us;
    f->now = f->now > end ? f->now : end;
    lg(f, "W ");
    return k == f->scan_fail_at ? ESP_ERR_TIMEOUT : ESP_OK;
}
static uint64_t f_now(void *c) { return ((fk_t *)c)->now; }
static void f_wait(void *c, uint64_t t) { fk_t *f = c; if (t > f->now) f->now = t; }

static fk_t f;
static uint8_t b0[16], b1[16];
static uint8_t *const bufs[2] = {b0, b1};
static epd_par_run_ops_t ops;

static void setup(void)
{
    memset(&f, 0, sizeof(f));
    f.now = 1000000;
    f.frame_us = 11660;
    f.compute_us = 3000;
    f.pg_fail_at = -1;
    f.scan_fail_at = -1;
    f.late_at = -1;
    f.stall_at = -1;
    ops = (epd_par_run_ops_t){.ctx = &f, .hv_on = f_hv_on, .hv_off = f_hv_off, .pg_ok = f_pg, .compute = f_compute,
                              .scan_start = f_scan_start, .scan_wait = f_scan_wait, .now_us = f_now,
                              .wait_until_us = f_wait};
}

static void test_run_order_double_buffer(void)
{
    setup();
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 3, 85, 1000, bufs, &st) == EPD_RUN_OK);
    // PWR_GOOD is read while the frame scans (an I2C read takes longer than
    // the 90 us between two frames of the 85 Hz timeline: device c1)
    CHECK(strcmp(f.log, "ON C0 S0 P C1 W S1 P C2 W S2 P W OFF ") == 0);
    CHECK(f.computed[0] == b0 && f.computed[1] == b1 && f.computed[2] == b0);
    CHECK(f.scanned[0] == b0 && f.scanned[1] == b1 && f.scanned[2] == b0);
    CHECK(st.phases_done == 3 && !st.unknown);
    CHECK(st.compute_max_us == 3000);
    CHECK(f.hv_offs == 1);
}

static void test_run_paces_phases(void)
{
    setup();
    f.frame_us = 2000;      // a bus faster than the phase rate must still wait
    f.compute_us = 100;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 5, 85, 1000, bufs, &st) == EPD_RUN_OK);
    // stock timeline (render_lcd.c:240-243): phase k at origin + ceil(k * 1e6 / 85)
    for (int k = 1; k < 5; k++) {
        CHECK(f.scan_t[k] - f.scan_t[0] == ((uint64_t)k * 1000000u + 84u) / 85u);
    }
    CHECK(st.late_phases == 0);
    // a late phase never moves the later targets, and no phase starts early
    setup();
    f.compute_us = 2000;
    f.frame_us = 2000;
    f.late_at = 2;          // phase 2 starts 20 ms late (compute stall)
    CHECK(epd_par_run(&ops, 6, 85, 1000, bufs, &st) == EPD_RUN_OK);
    CHECK(st.late_phases >= 1);
    for (int k = 1; k < 6; k++) {
        CHECK(f.scan_t[k] - f.scan_t[0] >= ((uint64_t)k * 1000000u + 84u) / 85u);
    }
    CHECK(f.scan_t[5] - f.scan_t[0] == (5u * 1000000u + 84u) / 85u);   // back on the timeline
}

static void test_run_pg_drop_aborts_and_powers_off(void)
{
    setup();
    f.pg_fail_at = 2;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 6, 85, 1000, bufs, &st) == EPD_RUN_PG_DROP);
    CHECK(f.hv_offs == 1);
    CHECK(st.phases_done == 2 && st.unknown);
    // the failed check is during frame 2: rails off at once (XOE first), then
    // that frame is let end; no further frame starts
    CHECK(strstr(f.log, "S2 p OFF W ") != NULL);
    const size_t n = strlen(f.log);
    CHECK(n > 2 && strcmp(f.log + n - 2, "W ") == 0);
    CHECK(strstr(f.log, "S3") == NULL);
}

static void test_run_phase_timing_stats(void)
{
    setup();
    f.compute_us = 3000;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 4, 85, 1000, bufs, &st) == EPD_RUN_OK);
    CHECK(st.lag_us_max <= 1);          // every phase on its slot
    CHECK(st.frame_us_max == 11660);    // scan_start to scan_wait return (fake frame)
}

static void test_run_scan_error_marks_unknown(void)
{
    setup();
    f.scan_fail_at = 0;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 6, 85, 1000, bufs, &st) == EPD_RUN_SCAN_FAIL);
    CHECK(f.scans == 1 && f.hv_offs == 1);
    CHECK(st.unknown && st.phases_done == 0);
}

// The budget is checked once, before phase 0: a run that cannot fit is not
// started (the panel keeps its image); 48 phases x 11.76 ms = 565 ms > 300 ms
static void test_run_budget_checked_before_phase0(void)
{
    setup();
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 48, 85, 300, bufs, &st) == EPD_RUN_BUDGET);
    CHECK(f.scans == 0 && f.hv_offs == 1);
    CHECK(!st.unknown && st.phases_done == 0);
}

// Device stage d: a GC16 cut after 15 phases by the budget left the panel in
// the waveform's dark middle (black page, white UI, ghosts). Once phase 0 has
// been scanned, no deadline stops a run; only real faults (scan error, PWR_GOOD
// drop, and the BSP high-voltage watchdog outside this loop) do.
static void test_run_never_cancelled_after_phase0(void)
{
    setup();
    f.stall_at = 10;
    f.stall_us = 600000;    // a 0.6 s stall in the middle
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 48, 85, 1020, bufs, &st) == EPD_RUN_OK);
    CHECK(st.phases_done == 48 && f.scans == 48 - (int)st.reused);
    CHECK(st.late_phases >= 1);     // reported, not fatal
}

static void test_run_hv_fail_no_scan(void)
{
    setup();
    f.hv_fail = true;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 6, 85, 1000, bufs, &st) == EPD_RUN_HV_FAIL);
    CHECK(f.scans == 0 && f.pg_checks == 0);
    CHECK(f.hv_offs == 1);          // still sequenced off
    CHECK(!st.unknown);
}

// A phase identical to the previous one scans the previous buffer again
static void test_run_reuses_identical_phase(void)
{
    setup();
    f.reuse_mask = (1u << 1) | (1u << 2);   // phases 1 and 2 repeat phase 0
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 4, 85, 1000, bufs, &st) == EPD_RUN_OK);
    CHECK(strcmp(f.log, "ON C0 S0 P R1 W S1 P R2 W S2 P C3 W S3 P W OFF ") == 0);
    CHECK(f.scanned[0] == b0 && f.scanned[1] == b0 && f.scanned[2] == b0);
    CHECK(f.computed[3] == b1 && f.scanned[3] == b1);   // never the buffer being scanned
    CHECK(st.reused == 2);
}

// Device stage d: a GC16 aborted with BUDGET after 14-15 phases because
// hv_on (PMIC wake from SLEEP, I2C, PWR_GOOD) took about 0.85 s at that
// moment. The budget limits the time with the rails up: it starts once
// hv_on has returned, and hv_on's own time is reported.
static void test_run_budget_starts_after_hv_on(void)
{
    setup();
    f.hv_on_us = 850000;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 48, 85, 1020, bufs, &st) == EPD_RUN_OK);
    CHECK(st.phases_done == 48);
    CHECK(st.hv_on_us == 850000);
}

// Plan 9 T6: rails held from the previous run of a burst. hv_on is skipped,
// and the run keeps the rails up for the next one when asked; every error
// still powers off.
static void test_run_keep_rails_skips_hv_on(void)
{
    setup();
    const epd_par_run_opts_t o = {.rails_on = true, .keep_rails = false};
    epd_par_run_stats_t st;
    CHECK(epd_par_run_ex(&ops, 2, 85, 1000, bufs, &o, &st) == EPD_RUN_OK);
    CHECK(strcmp(f.log, "C0 S0 P C1 W S1 P W OFF ") == 0);
    CHECK(f.hv_ons == 0 && f.hv_offs == 1 && st.hv_on_us == 0);
}

static void test_run_keep_rails_leaves_on(void)
{
    setup();
    const epd_par_run_opts_t o = {.rails_on = false, .keep_rails = true};
    epd_par_run_stats_t st;
    CHECK(epd_par_run_ex(&ops, 2, 85, 1000, bufs, &o, &st) == EPD_RUN_OK);
    CHECK(strcmp(f.log, "ON C0 S0 P C1 W S1 P W ") == 0);
    CHECK(f.hv_offs == 0 && !st.unknown && st.phases_done == 2);
    // the next run of the burst: neither on nor off
    memset(f.log, 0, sizeof(f.log));
    const epd_par_run_opts_t o2 = {.rails_on = true, .keep_rails = true};
    CHECK(epd_par_run_ex(&ops, 1, 85, 1000, bufs, &o2, &st) == EPD_RUN_OK);
    CHECK(strcmp(f.log, "C0 S2 P W ") == 0);     // scans count on from the first run
    CHECK(f.hv_ons == 1 && f.hv_offs == 0);
}

// With the rails already up the budget starts at the call: a run that does
// not fit is refused before phase 0, and the rails go off (nothing scanned)
static void test_run_budget_with_rails_on(void)
{
    setup();
    const epd_par_run_opts_t o = {.rails_on = true, .keep_rails = true};
    epd_par_run_stats_t st;
    CHECK(epd_par_run_ex(&ops, 48, 85, 300, bufs, &o, &st) == EPD_RUN_BUDGET);
    CHECK(f.scans == 0 && f.hv_ons == 0 && f.hv_offs == 1);
    CHECK(!st.unknown);
    setup();
    CHECK(epd_par_run_ex(&ops, 20, 85, 600, bufs, &o, &st) == EPD_RUN_OK);
    CHECK(st.phases_done == 20 && f.hv_offs == 0);
}

static void test_run_error_turns_rails_off_even_with_keep(void)
{
    const epd_par_run_opts_t o = {.rails_on = true, .keep_rails = true};
    epd_par_run_stats_t st;
    setup();
    f.pg_fail_at = 1;
    CHECK(epd_par_run_ex(&ops, 6, 85, 1000, bufs, &o, &st) == EPD_RUN_PG_DROP);
    CHECK(f.hv_offs == 1 && st.unknown);
    setup();
    f.scan_fail_at = 2;
    CHECK(epd_par_run_ex(&ops, 6, 85, 1000, bufs, &o, &st) == EPD_RUN_SCAN_FAIL);
    CHECK(f.hv_offs == 1 && st.unknown);
    setup();
    f.hv_fail = true;
    const epd_par_run_opts_t o3 = {.rails_on = false, .keep_rails = true};
    CHECK(epd_par_run_ex(&ops, 6, 85, 1000, bufs, &o3, &st) == EPD_RUN_HV_FAIL);
    CHECK(f.hv_offs == 1 && f.scans == 0);
}

int main(void)
{
    test_run_order_double_buffer();
    test_run_paces_phases();
    test_run_pg_drop_aborts_and_powers_off();
    test_run_scan_error_marks_unknown();
    test_run_budget_checked_before_phase0();
    test_run_never_cancelled_after_phase0();
    test_run_budget_starts_after_hv_on();
    test_run_hv_fail_no_scan();
    test_run_reuses_identical_phase();
    test_run_phase_timing_stats();
    test_run_keep_rails_skips_hv_on();
    test_run_keep_rails_leaves_on();
    test_run_budget_with_rails_on();
    test_run_error_turns_rails_off_even_with_keep();
    TEST_DONE("par_run");
}
