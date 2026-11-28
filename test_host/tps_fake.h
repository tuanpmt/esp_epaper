#pragma once

// Fake TPS65185 + control lines for host tests: records every operation with
// a simulated clock, models PWR_GOOD timing, NACKs while WAKEUP is low.

#include <stdbool.h>
#include <stdint.h>
#include "epd_tps65185.h"

typedef enum { TF_PROBE, TF_RD, TF_WR, TF_LINE, TF_PG, TF_OE, TF_DELAY, TF_BUS } tf_kind_t;

typedef struct {
    tf_kind_t k;
    uint8_t a, b;       // RD/WR: reg, value; LINE: line, level; PG: level
    uint32_t us;        // DELAY: duration
    uint64_t t_us;
} tf_ev_t;

typedef struct {
    tf_ev_t ev[2048];
    int n;
    uint64_t now_us;
    bool line[3];
    uint8_t regs[0x11];
    int64_t pg_rise_us;     // PWR_GOOD pin goes high this long after PWRUP rises; < 0 never
    int64_t pg_fall_us;     // and low this long after PWRUP falls
    uint64_t pwrup_edge_us;
    bool was_good;          // PWR_GOOD was high when PWRUP fell
    bool pg_reg_good;       // PG register reads 0xFA once the pin is good
    int fail_i2c_at;        // fail the n-th I2C operation (1-based), 0 never
    int i2c_ops;
    int asleep_i2c;         // I2C attempts while WAKEUP was low
    int tmst_polls;         // TMST1 reads before the done bit shows
    bool pg_drop;           // PWR_GOOD pin forced low (fault injection)
    bool int_latched;       // a fault latched INT bits: rails stay off until INT1 and INT2 are read (8.3.8)
    bool int1_read;
    bool bus_powered;       // last bus_idle(): SPV/XSTL at their powered idle (high)
    epd_tps_ops_t ops;
} tps_fake_t;

void tf_init(tps_fake_t *f);
bool tf_pin_good(tps_fake_t *f);
int tf_find(const tps_fake_t *f, tf_kind_t k, int a, int b, int from);   // index or -1; a/b -1 = any
int tf_count(const tps_fake_t *f, tf_kind_t k, int a, int b);
