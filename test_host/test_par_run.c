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
    int pg_checks, scans, hv_offs, hv_ons;
    const uint8_t *scanned[64];
    uint64_t scan_t[64];
    uint8_t *computed[64];
} fk_t;

static void lg(fk_t *f, const char *s)
{
    strncat(f->log, s, sizeof(f->log) - strlen(f->log) - 1);
}

static esp_err_t f_hv_on(void *c) { fk_t *f = c; f->hv_ons++; lg(f, "ON "); return f->hv_fail ? ESP_FAIL : ESP_OK; }
static esp_err_t f_hv_off(void *c) { fk_t *f = c; f->hv_offs++; lg(f, "OFF "); return ESP_OK; }
static bool f_pg(void *c)
{
    fk_t *f = c;
    const bool ok = f->pg_checks != f->pg_fail_at;
    f->pg_checks++;
    lg(f, ok ? "P " : "p ");
    return ok;
}
static void f_compute(void *c, int phase, uint8_t *buf)
{
    fk_t *f = c;
    char s[16];
    snprintf(s, sizeof(s), "C%d ", phase);
    lg(f, s);
    f->computed[phase] = buf;
    f->now += f->compute_us;
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
    ops = (epd_par_run_ops_t){.ctx = &f, .hv_on = f_hv_on, .hv_off = f_hv_off, .pg_ok = f_pg, .compute = f_compute,
                              .scan_start = f_scan_start, .scan_wait = f_scan_wait, .now_us = f_now,
                              .wait_until_us = f_wait};
}

static void test_run_order_double_buffer(void)
{
    setup();
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 3, 11764, 1000, bufs, &st) == EPD_RUN_OK);
    CHECK(strcmp(f.log, "ON C0 P S0 C1 W P S1 C2 W P S2 W OFF ") == 0);
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
    CHECK(epd_par_run(&ops, 5, 11764, 1000, bufs, &st) == EPD_RUN_OK);
    for (int k = 1; k < 5; k++) {
        CHECK(f.scan_t[k] - f.scan_t[k - 1] >= 11764);
        CHECK(f.scan_t[k] - f.scan_t[k - 1] < 11764 + 50);
    }
    CHECK(st.late_phases == 0);
    // a slow compute makes phases late, never early
    setup();
    f.compute_us = 20000;
    CHECK(epd_par_run(&ops, 3, 11764, 1000, bufs, &st) == EPD_RUN_OK);
    CHECK(st.late_phases >= 1);
    for (int k = 1; k < 3; k++) {
        CHECK(f.scan_t[k] - f.scan_t[k - 1] >= 11764);
    }
}

static void test_run_pg_drop_aborts_and_powers_off(void)
{
    setup();
    f.pg_fail_at = 2;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 6, 11764, 1000, bufs, &st) == EPD_RUN_PG_DROP);
    CHECK(f.scans == 2);            // no scan after the failed check
    CHECK(f.hv_offs == 1);
    CHECK(st.phases_done == 2 && st.unknown);
    const size_t n = strlen(f.log);
    CHECK(n > 4 && strcmp(f.log + n - 4, "OFF ") == 0);
}

static void test_run_scan_error_marks_unknown(void)
{
    setup();
    f.scan_fail_at = 0;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 6, 11764, 1000, bufs, &st) == EPD_RUN_SCAN_FAIL);
    CHECK(f.scans == 1 && f.hv_offs == 1);
    CHECK(st.unknown && st.phases_done == 0);
}

static void test_run_budget_exceeded_aborts(void)
{
    setup();
    epd_par_run_stats_t st;
    // 48 phases x 11.76 ms = 565 ms > 300 ms budget
    CHECK(epd_par_run(&ops, 48, 11764, 300, bufs, &st) == EPD_RUN_BUDGET);
    CHECK(f.hv_offs == 1 && st.unknown);
    CHECK(st.hv_us <= 300000 + 11764 + 3000);
    CHECK(st.phases_done < 48 && st.phases_done >= 24);
}

static void test_run_hv_fail_no_scan(void)
{
    setup();
    f.hv_fail = true;
    epd_par_run_stats_t st;
    CHECK(epd_par_run(&ops, 6, 11764, 1000, bufs, &st) == EPD_RUN_HV_FAIL);
    CHECK(f.scans == 0 && f.pg_checks == 0);
    CHECK(f.hv_offs == 1);          // still sequenced off
    CHECK(!st.unknown);
}

int main(void)
{
    test_run_order_double_buffer();
    test_run_paces_phases();
    test_run_pg_drop_aborts_and_powers_off();
    test_run_scan_error_marks_unknown();
    test_run_budget_exceeded_aborts();
    test_run_hv_fail_no_scan();
    TEST_DONE("par_run");
}
