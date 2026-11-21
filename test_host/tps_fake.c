#include <string.h>
#include "tps_fake.h"

static void rec(tps_fake_t *f, tf_kind_t k, int a, int b)
{
    if (f->n < (int)(sizeof(f->ev) / sizeof(f->ev[0]))) {
        f->ev[f->n++] = (tf_ev_t){.k = k, .a = (uint8_t)a, .b = (uint8_t)b, .t_us = f->now_us};
    }
}

bool tf_pin_good(tps_fake_t *f)
{
    if (f->pg_drop || !f->line[EPD_TPS_LINE_WAKEUP]) {
        return false;
    }
    const uint64_t since = f->now_us - f->pwrup_edge_us;
    if (f->line[EPD_TPS_LINE_PWRUP]) {
        return f->pg_rise_us >= 0 && since >= (uint64_t)f->pg_rise_us;
    }
    // falling: still good until pg_fall_us after PWRUP went low (only if it was ever good)
    return f->was_good && f->pg_fall_us > 0 && since < (uint64_t)f->pg_fall_us;
}

static esp_err_t i2c_gate(tps_fake_t *f)
{
    f->i2c_ops++;
    if (!f->line[EPD_TPS_LINE_WAKEUP]) {
        f->asleep_i2c++;
        return ESP_FAIL;    // NACK
    }
    if (f->fail_i2c_at && f->i2c_ops == f->fail_i2c_at) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t f_probe(void *ctx)
{
    tps_fake_t *f = ctx;
    rec(f, TF_PROBE, 0, 0);
    return i2c_gate(f);
}

static esp_err_t f_read(void *ctx, uint8_t reg, uint8_t *val)
{
    tps_fake_t *f = ctx;
    esp_err_t err = i2c_gate(f);
    if (err == ESP_OK) {
        if (reg == EPD_TPS_REG_PG) {
            f->regs[reg] = (f->pg_reg_good && tf_pin_good(f)) ? 0xFA : 0x00;
        } else if (reg == EPD_TPS_REG_TMST1) {
            if (f->tmst_polls > 0) {
                f->tmst_polls--;
            } else {
                f->regs[reg] |= EPD_TPS_TMST1_DONE;
            }
        }
        *val = reg <= EPD_TPS_REG_REVID ? f->regs[reg] : 0;
    }
    rec(f, TF_RD, reg, err == ESP_OK ? *val : 0xEE);
    return err;
}

static esp_err_t f_write(void *ctx, uint8_t reg, uint8_t val)
{
    tps_fake_t *f = ctx;
    esp_err_t err = i2c_gate(f);
    rec(f, TF_WR, reg, val);
    if (err == ESP_OK && reg <= EPD_TPS_REG_REVID && reg != EPD_TPS_REG_REVID) {
        f->regs[reg] = (reg == EPD_TPS_REG_TMST1) ? (uint8_t)(val & ~EPD_TPS_TMST1_DONE) : val;
    }
    return err;
}

static esp_err_t f_line(void *ctx, epd_tps_line_t line, bool level)
{
    tps_fake_t *f = ctx;
    rec(f, TF_LINE, line, level);
    if (line == EPD_TPS_LINE_PWRUP && level != f->line[line]) {
        f->was_good = !level && tf_pin_good(f);
        f->pwrup_edge_us = f->now_us;
    }
    f->line[line] = level;
    return ESP_OK;
}

static esp_err_t f_pg(void *ctx, bool *good)
{
    tps_fake_t *f = ctx;
    *good = tf_pin_good(f);
    rec(f, TF_PG, *good, 0);
    return ESP_OK;
}

static void f_oe(void *ctx)
{
    tps_fake_t *f = ctx;
    rec(f, TF_OE, 0, 0);
}

static void f_delay(void *ctx, uint32_t us)
{
    tps_fake_t *f = ctx;
    rec(f, TF_DELAY, 0, 0);
    f->ev[f->n - 1].us = us;
    f->now_us += us;
}

void tf_init(tps_fake_t *f)
{
    memset(f, 0, sizeof(*f));
    f->regs[EPD_TPS_REG_REVID] = EPD_TPS_REVID;
    f->regs[EPD_TPS_REG_INT2] = 0x01;
    f->regs[EPD_TPS_REG_TMST_VALUE] = 27;
    f->regs[EPD_TPS_REG_UPSEQ0] = 0xE4;
    f->regs[EPD_TPS_REG_UPSEQ1] = 0x55;
    f->pg_rise_us = 20000;
    f->pg_fall_us = 5000;
    f->pg_reg_good = true;
    f->now_us = 1000000;
    f->ops = (epd_tps_ops_t){.ctx = f, .probe = f_probe, .reg_read = f_read, .reg_write = f_write,
                             .line_set = f_line, .pg_pin = f_pg, .oe_low = f_oe, .delay_us = f_delay};
}

int tf_find(const tps_fake_t *f, tf_kind_t k, int a, int b, int from)
{
    for (int i = from < 0 ? 0 : from; i < f->n; i++) {
        if (f->ev[i].k == k && (a < 0 || f->ev[i].a == a) && (b < 0 || f->ev[i].b == b)) {
            return i;
        }
    }
    return -1;
}

int tf_count(const tps_fake_t *f, tf_kind_t k, int a, int b)
{
    int n = 0;
    for (int i = tf_find(f, k, a, b, 0); i >= 0; i = tf_find(f, k, a, b, i + 1)) {
        n++;
    }
    return n;
}
