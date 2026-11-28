// TPS65185 sequencer: order, refusals and fault paths against a fake PMIC.
#include <string.h>
#include "check.h"
#include "epd_tps65185.h"
#include "tps_fake.h"

#define L_VCOM EPD_TPS_LINE_VCOM_CTRL
#define L_PWRUP EPD_TPS_LINE_PWRUP
#define L_WAKE EPD_TPS_LINE_WAKEUP

static tps_fake_t f;
static epd_tps_t t;

static void setup(void)
{
    tf_init(&f);
    CHECK(epd_tps_init(&t, &f.ops, 1200) == ESP_OK);
    CHECK(f.n == 0);    // init touches nothing
}

static void test_tps_vcom_encode(void)
{
    uint8_t v1, v2;
    epd_tps_vcom_encode(1200, &v1, &v2);
    CHECK(v1 == 0x78 && v2 == 0x00);
    epd_tps_vcom_encode(3000, &v1, &v2);
    CHECK(v1 == 0x2C && v2 == 0x01);
    epd_tps_vcom_encode(2560, &v1, &v2);
    CHECK(v1 == 0x00 && v2 == 0x01);
}

static void test_tps_hv_on_order(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_state(&t) == EPD_TPS_ON);
    const int oe = tf_find(&f, TF_OE, -1, -1, 0);
    const int wake = tf_find(&f, TF_LINE, L_WAKE, 1, 0);
    const int probe = tf_find(&f, TF_PROBE, -1, -1, 0);
    const int rev = tf_find(&f, TF_RD, EPD_TPS_REG_REVID, -1, 0);
    const int up0 = tf_find(&f, TF_WR, EPD_TPS_REG_UPSEQ0, 0xE4, 0);
    const int up1 = tf_find(&f, TF_WR, EPD_TPS_REG_UPSEQ1, 0x55, 0);
    const int vc2 = tf_find(&f, TF_WR, EPD_TPS_REG_VCOM2, 0x00, 0);
    const int vc1 = tf_find(&f, TF_WR, EPD_TPS_REG_VCOM1, 0x78, 0);
    const int pwrup = tf_find(&f, TF_LINE, L_PWRUP, 1, 0);
    const int pg = tf_find(&f, TF_PG, 1, -1, 0);
    const int en = tf_find(&f, TF_WR, EPD_TPS_REG_ENABLE, EPD_TPS_ENABLE_RAILS, 0);
    const int pgreg = tf_find(&f, TF_RD, EPD_TPS_REG_PG, 0xFA, 0);
    const int vcom = tf_find(&f, TF_LINE, L_VCOM, 1, 0);
    CHECK(oe >= 0 && oe < wake);
    CHECK(wake >= 0 && wake < probe && probe < rev);
    CHECK(f.ev[probe].t_us - f.ev[wake].t_us >= EPD_TPS_WAKE_MS * 1000u);
    CHECK(rev < up0 && up0 < up1);
    CHECK(vc1 >= 0 && vc2 >= 0 && up1 < vc1 && up1 < vc2);
    CHECK(vc1 < pwrup && vc2 < pwrup);      // VCOM programmed before any rail
    CHECK(pwrup < pg && pg < en && en < pgreg && pgreg < vcom);
    CHECK(vcom == f.n - 1 || tf_find(&f, TF_WR, -1, -1, vcom) < 0);   // nothing written after VCOM_CTRL
    // read-backs of UPSEQ and VCOM happened
    CHECK(tf_find(&f, TF_RD, EPD_TPS_REG_UPSEQ1, -1, up1) > up1);
    CHECK(tf_find(&f, TF_RD, EPD_TPS_REG_VCOM1, -1, vc1) > vc1);
    CHECK(f.asleep_i2c == 0);
    CHECK(epd_tps_stats(&t)->pg_ms >= 19 && epd_tps_stats(&t)->pg_ms <= 22);
    CHECK(epd_tps_stats(&t)->on_count == 1);
}

static void test_tps_pg_timeout_never_enables(void)
{
    setup();
    f.pg_rise_us = -1;      // PWR_GOOD never comes
    CHECK(epd_tps_hv_on(&t) == ESP_ERR_TIMEOUT);
    CHECK(tf_count(&f, TF_WR, EPD_TPS_REG_ENABLE, -1) == 0);
    CHECK(tf_count(&f, TF_LINE, L_VCOM, 1) == 0);
    // every PWRUP=1 is followed by PWRUP=0, and the last word on PWRUP is low
    CHECK(tf_count(&f, TF_LINE, L_PWRUP, 0) >= tf_count(&f, TF_LINE, L_PWRUP, 1));
    CHECK(!f.line[L_PWRUP] && !f.line[L_VCOM]);
    // the PG wait is bounded: 100 ms per attempt
    const int up = tf_find(&f, TF_LINE, L_PWRUP, 1, 0);
    const int down = tf_find(&f, TF_LINE, L_PWRUP, 0, up);
    CHECK(down > up);
    const uint64_t waited = f.ev[down].t_us - f.ev[up].t_us;
    CHECK(waited >= EPD_TPS_PG_TIMEOUT_MS * 1000u && waited < (EPD_TPS_PG_TIMEOUT_MS + 5) * 1000u);
}

static void test_tps_retry_then_failed_latch(void)
{
    setup();
    f.pg_rise_us = -1;
    CHECK(epd_tps_hv_on(&t) == ESP_ERR_TIMEOUT);
    CHECK(tf_count(&f, TF_LINE, L_PWRUP, 1) == 2);   // one retry
    CHECK(epd_tps_state(&t) == EPD_TPS_FAILED);
    const int n = f.n;
    CHECK(epd_tps_hv_on(&t) == ESP_ERR_INVALID_STATE);
    CHECK(tf_count(&f, TF_LINE, L_PWRUP, 1) == 2);   // latched: no third attempt
    CHECK(epd_tps_hv_off(&t) == ESP_OK);              // power-down is always allowed
    CHECK(f.n > n);
    CHECK(epd_tps_state(&t) == EPD_TPS_FAILED);
    // a retry that works resets the failure count
    setup();
    // first attempt: I2C error on the UPSEQ0 write (probe, REVID, INT1, INT2, then write)
    f.fail_i2c_at = 5;
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_state(&t) == EPD_TPS_ON);
    CHECK(epd_tps_stats(&t)->fail_count == 1);
}

static void test_tps_thermal_flag_refuses_wake(void)
{
    setup();
    f.regs[EPD_TPS_REG_INT1] = 0x40;     // TSD
    CHECK(epd_tps_wake(&t) != ESP_OK);
    CHECK(!f.line[L_WAKE]);
    CHECK(tf_count(&f, TF_LINE, L_PWRUP, 1) == 0);
    CHECK(epd_tps_stats(&t)->int1 == 0x40);
}

static void test_tps_revid_mismatch_refuses(void)
{
    setup();
    f.regs[EPD_TPS_REG_REVID] = 0x66;
    CHECK(epd_tps_hv_on(&t) != ESP_OK);
    CHECK(tf_count(&f, TF_LINE, L_PWRUP, 1) == 0);
    CHECK(!f.line[L_WAKE]);
}

static void test_tps_vcom2_guard_refuses_high_bits(void)
{
    setup();
    CHECK(epd_tps_wake(&t) == ESP_OK);
    const int n = f.n;
    for (int bit = 1; bit < 8; bit++) {
        CHECK(epd_tps_reg_write(&t, EPD_TPS_REG_VCOM2, (uint8_t)(1u << bit)) == ESP_ERR_NOT_ALLOWED);
    }
    CHECK(epd_tps_reg_write(&t, EPD_TPS_REG_ENABLE, 0x3F) == ESP_ERR_NOT_ALLOWED);
    CHECK(f.n == n);    // nothing reached the bus
    CHECK(epd_tps_reg_write(&t, EPD_TPS_REG_VCOM2, 0x01) == ESP_OK);
    CHECK(epd_tps_reg_write(&t, EPD_TPS_REG_INT_EN1, 0x00) == ESP_OK);
}

static void test_tps_vcom_range(void)
{
    setup();
    CHECK(epd_tps_set_vcom(&t, 999) == ESP_ERR_INVALID_ARG);
    CHECK(epd_tps_set_vcom(&t, 3001) == ESP_ERR_INVALID_ARG);
    CHECK(f.n == 0);
    CHECK(epd_tps_set_vcom(&t, 1600) == ESP_OK);     // asleep: stored, no I2C
    CHECK(f.n == 0);
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(tf_find(&f, TF_WR, EPD_TPS_REG_VCOM1, 160, 0) >= 0);
    epd_tps_t t2;
    CHECK(epd_tps_init(&t2, &f.ops, 500) == ESP_ERR_INVALID_ARG);
}

static void test_tps_vcom_refused_while_on(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    const int n = f.n;
    CHECK(epd_tps_set_vcom(&t, 1600) == ESP_ERR_INVALID_STATE);
    CHECK(f.n == n);
    CHECK(epd_tps_hv_off(&t) == ESP_OK);
    CHECK(epd_tps_set_vcom(&t, 1600) == ESP_OK);     // STANDBY: written and read back now
    CHECK(tf_find(&f, TF_WR, EPD_TPS_REG_VCOM1, 160, n) >= 0);
}

static void test_tps_hv_off_order(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    const int n = f.n;
    CHECK(epd_tps_hv_off(&t) == ESP_OK);
    const int oe = tf_find(&f, TF_OE, -1, -1, n);
    const int vcom = tf_find(&f, TF_LINE, L_VCOM, 0, n);
    const int pwr = tf_find(&f, TF_LINE, L_PWRUP, 0, n);
    CHECK(oe >= 0 && oe < vcom && vcom < pwr);
    CHECK(f.ev[vcom].t_us - f.ev[oe].t_us >= EPD_TPS_OE_LEAD_US);
    CHECK(tf_find(&f, TF_PG, 0, -1, pwr) > pwr);      // waited for PWR_GOOD low
    CHECK(f.line[L_WAKE]);                            // stays STANDBY: next refresh skips the wake
    CHECK(epd_tps_state(&t) == EPD_TPS_STANDBY);
    CHECK(epd_tps_stats(&t)->off_pg_ms >= 4 && epd_tps_stats(&t)->off_pg_ms <= 6);
}

static void test_tps_hv_off_idempotent(void)
{
    setup();
    CHECK(epd_tps_hv_off(&t) == ESP_OK);     // asleep: lines only, no I2C, no PG read
    CHECK(f.i2c_ops == 0 && tf_count(&f, TF_PG, -1, -1) == 0);
    CHECK(epd_tps_state(&t) == EPD_TPS_SLEEP);
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_hv_off(&t) == ESP_OK);
    CHECK(epd_tps_hv_off(&t) == ESP_OK);
    CHECK(epd_tps_state(&t) == EPD_TPS_STANDBY);
    CHECK(!f.line[L_PWRUP] && !f.line[L_VCOM]);
}

static void test_tps_no_i2c_while_asleep(void)
{
    setup();
    uint8_t v;
    int8_t c;
    CHECK(epd_tps_reg_read(&t, EPD_TPS_REG_REVID, &v) == ESP_ERR_INVALID_STATE);
    CHECK(epd_tps_reg_write(&t, EPD_TPS_REG_INT_EN1, 0) == ESP_ERR_INVALID_STATE);
    CHECK(epd_tps_temperature(&t, &c) == ESP_ERR_INVALID_STATE);
    CHECK(!epd_tps_pg_ok(&t));
    CHECK(f.i2c_ops == 0 && f.asleep_i2c == 0);
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_sleep(&t) == ESP_OK);
    CHECK(epd_tps_reg_read(&t, EPD_TPS_REG_REVID, &v) == ESP_ERR_INVALID_STATE);
    CHECK(f.asleep_i2c == 0);
}

static void test_tps_sleep_waits_120ms(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    const int n = f.n;
    CHECK(epd_tps_sleep(&t) == ESP_OK);
    const int pwr = tf_find(&f, TF_LINE, L_PWRUP, 0, n);
    const int wake = tf_find(&f, TF_LINE, L_WAKE, 0, n);
    CHECK(pwr >= 0 && pwr < wake);
    CHECK(f.now_us - f.ev[wake].t_us >= EPD_TPS_SLEEP_MS * 1000u);
    CHECK(epd_tps_state(&t) == EPD_TPS_SLEEP);
    CHECK(!f.line[L_WAKE] && !f.line[L_PWRUP] && !f.line[L_VCOM]);
}

static void test_tps_temperature(void)
{
    setup();
    CHECK(epd_tps_wake(&t) == ESP_OK);
    f.tmst_polls = 3;
    int8_t c = 0;
    CHECK(epd_tps_temperature(&t, &c) == ESP_OK);
    CHECK(c == 27);
    CHECK(tf_find(&f, TF_WR, EPD_TPS_REG_TMST1, EPD_TPS_TMST1_READ, 0) >= 0);
    f.tmst_polls = 1000;
    CHECK(epd_tps_temperature(&t, &c) == ESP_ERR_TIMEOUT);
}

static void test_tps_pg_ok_drop_powers_off(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_pg_ok(&t));
    f.pg_drop = true;
    CHECK(!epd_tps_pg_ok(&t));
    CHECK(!f.line[L_PWRUP] && !f.line[L_VCOM]);
    CHECK(epd_tps_state(&t) == EPD_TPS_STANDBY);
}

// SLVSAQ8G 8.3.8: after a fault the rails cannot come up again until INT1 and
// INT2 were read; the chip stays in STANDBY between refreshes, so every
// power-up reads them first
static void test_tps_reads_int_before_every_power_up(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_hv_off(&t) == ESP_OK);
    f.int_latched = true;       // a fault during the last refresh
    f.int1_read = false;
    const int ups = tf_count(&f, TF_LINE, L_PWRUP, 1);
    const int n = f.n;
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(tf_count(&f, TF_LINE, L_PWRUP, 1) == ups + 1);    // first attempt, no retry
    const int i1 = tf_find(&f, TF_RD, EPD_TPS_REG_INT1, -1, n);
    const int i2 = tf_find(&f, TF_RD, EPD_TPS_REG_INT2, -1, n);
    const int up = tf_find(&f, TF_LINE, L_PWRUP, 1, n);
    CHECK(i1 >= 0 && i2 > i1 && up > i2);
}

// Device round a2: rails failed with VEE_UV while every bus pin was low. The
// vendor firmware powers up with SPV and XSTL (both active low) high: the bus
// goes to its powered-idle levels before WAKEUP and back to all-low in SLEEP.
static void test_tps_bus_idle_before_power_up(void)
{
    setup();
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    const int bus = tf_find(&f, TF_BUS, 1, -1, 0);
    const int wake = tf_find(&f, TF_LINE, L_WAKE, 1, 0);
    const int up = tf_find(&f, TF_LINE, L_PWRUP, 1, 0);
    CHECK(bus >= 0 && bus < wake && wake < up);
    CHECK(f.bus_powered);
    CHECK(epd_tps_hv_off(&t) == ESP_OK);
    CHECK(f.bus_powered);                   // STANDBY between refreshes: idle levels stay
    CHECK(epd_tps_sleep(&t) == ESP_OK);
    const int down = tf_find(&f, TF_LINE, L_WAKE, 0, 0);
    const int rest = tf_find(&f, TF_BUS, 0, -1, down);
    CHECK(rest > down && !f.bus_powered);   // V3P3 is off in SLEEP: nothing driven high
}

// The PMIC gives each rail 50 ms (SLVSAQ8G 8.3.2) plus the UPSEQ1 delays; the
// vendor firmware waits 50 + 200 ms: a slow but healthy ramp must not time out
static void test_tps_pg_wait_covers_slow_ramp(void)
{
    setup();
    f.pg_rise_us = 220000;
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    CHECK(epd_tps_stats(&t)->fail_count == 0);
}

static void test_tps_budget(void)
{
    CHECK(epd_tps_hv_budget_ms(20) == 600);
    CHECK(epd_tps_hv_budget_ms(48) == 1020);
    CHECK(epd_tps_hv_budget_ms(200) == 2000);
    CHECK(epd_tps_hv_budget_ms(0) == 300);
}

// TPS65185 registers reset in SLEEP: every power-up from SLEEP must write
// VCOM (and UPSEQ) again, not only the first one
static void test_tps_vcom_written_after_every_sleep(void)
{
    setup();
    for (int i = 0; i < 3; i++) {
        const int n = f.n;
        CHECK(epd_tps_hv_on(&t) == ESP_OK);
        CHECK(tf_find(&f, TF_WR, EPD_TPS_REG_VCOM1, 0x78, n) >= 0);
        CHECK(tf_find(&f, TF_WR, EPD_TPS_REG_VCOM2, 0x00, n) >= 0);
        CHECK(tf_find(&f, TF_WR, EPD_TPS_REG_UPSEQ0, 0xE4, n) >= 0);
        CHECK(epd_tps_sleep(&t) == ESP_OK);
    }
}

// Register dump for the console: every register but INT1/INT2 (clear on read)
static void test_tps_snapshot(void)
{
    setup();
    uint8_t r[EPD_TPS_REG_REVID + 1];
    CHECK(epd_tps_snapshot(&t, r) == ESP_ERR_INVALID_STATE);    // asleep: no I2C
    CHECK(epd_tps_hv_on(&t) == ESP_OK);
    const int n = f.n;
    CHECK(epd_tps_snapshot(&t, r) == ESP_OK);
    CHECK(tf_find(&f, TF_RD, EPD_TPS_REG_INT1, -1, n) < 0);
    CHECK(tf_find(&f, TF_RD, EPD_TPS_REG_INT2, -1, n) < 0);
    CHECK(r[EPD_TPS_REG_INT1] == 0xFF && r[EPD_TPS_REG_INT2] == 0xFF);
    CHECK(r[EPD_TPS_REG_VCOM1] == 0x78 && r[EPD_TPS_REG_VCOM2] == 0x00);
    CHECK(r[EPD_TPS_REG_UPSEQ0] == 0xE4 && r[EPD_TPS_REG_REVID] == EPD_TPS_REVID);
}

int main(void)
{
    test_tps_vcom_encode();
    test_tps_hv_on_order();
    test_tps_pg_timeout_never_enables();
    test_tps_retry_then_failed_latch();
    test_tps_thermal_flag_refuses_wake();
    test_tps_revid_mismatch_refuses();
    test_tps_vcom2_guard_refuses_high_bits();
    test_tps_vcom_range();
    test_tps_vcom_refused_while_on();
    test_tps_hv_off_order();
    test_tps_hv_off_idempotent();
    test_tps_no_i2c_while_asleep();
    test_tps_sleep_waits_120ms();
    test_tps_temperature();
    test_tps_pg_ok_drop_powers_off();
    test_tps_reads_int_before_every_power_up();
    test_tps_bus_idle_before_power_up();
    test_tps_pg_wait_covers_slow_ramp();
    test_tps_budget();
    test_tps_vcom_written_after_every_sleep();
    test_tps_snapshot();
    TEST_DONE("tps65185");
}
