// TPS65185 power sequencer. See epd_tps65185.h for the order and the rules.
// Register values follow the vendor driver (CloudZao Metalio-E-INK4-Plus,
// main/boards/metalio-e-ink4-plus/bus/tps65185.c and the board file, MIT).

#include <stddef.h>
#include "epd_tps65185.h"

#define POLL_US 1000u

static void delay_us(epd_tps_t *t, uint32_t us)
{
    t->ops->delay_us(t->ops->ctx, us);
}

static esp_err_t line(epd_tps_t *t, epd_tps_line_t l, bool level)
{
    return t->ops->line_set(t->ops->ctx, l, level);
}

static bool awake(const epd_tps_t *t)
{
    return t->state == EPD_TPS_STANDBY || t->state == EPD_TPS_ON;
}

static esp_err_t rd(epd_tps_t *t, uint8_t reg, uint8_t *v)
{
    return t->ops->reg_read(t->ops->ctx, reg, v);
}

// The only path to the chip's registers. ENABLE passes only with allow_enable
// (the power-up step after PWR_GOOD); VCOM2 never carries bits 7..1.
static esp_err_t wr(epd_tps_t *t, uint8_t reg, uint8_t v, bool allow_enable)
{
    if (reg == EPD_TPS_REG_VCOM2 && (v & (uint8_t)~EPD_TPS_VCOM2_ALLOWED)) {
        return ESP_ERR_NOT_ALLOWED;
    }
    if (reg == EPD_TPS_REG_ENABLE && !allow_enable) {
        return ESP_ERR_NOT_ALLOWED;
    }
    return t->ops->reg_write(t->ops->ctx, reg, v);
}

static esp_err_t wr_verify(epd_tps_t *t, uint8_t reg, uint8_t v)
{
    esp_err_t err = wr(t, reg, v, false);
    uint8_t back = 0;
    if (err == ESP_OK) {
        err = rd(t, reg, &back);
    }
    if (err == ESP_OK && back != v) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

void epd_tps_vcom_encode(uint16_t mv, uint8_t *vcom1, uint8_t *vcom2)
{
    const unsigned v = mv / 10u;
    *vcom1 = (uint8_t)(v & 0xFFu);
    *vcom2 = (uint8_t)((v >> 8) & EPD_TPS_VCOM2_ALLOWED);
}

static esp_err_t write_vcom(epd_tps_t *t)
{
    uint8_t v1, v2;
    epd_tps_vcom_encode(t->vcom_mv, &v1, &v2);
    esp_err_t err = wr_verify(t, EPD_TPS_REG_VCOM2, v2);
    if (err == ESP_OK) {
        err = wr_verify(t, EPD_TPS_REG_VCOM1, v1);
    }
    return err;
}

uint32_t epd_tps_hv_budget_ms(uint16_t phases)
{
    const uint32_t b = (uint32_t)phases * 15u + 300u;
    return b > 2000u ? 2000u : b;
}

esp_err_t epd_tps_init(epd_tps_t *t, const epd_tps_ops_t *ops, uint16_t vcom_mv)
{
    if (!t || !ops || vcom_mv < EPD_TPS_VCOM_MIN_MV || vcom_mv > EPD_TPS_VCOM_MAX_MV) {
        return ESP_ERR_INVALID_ARG;
    }
    *t = (epd_tps_t){.ops = ops, .state = EPD_TPS_SLEEP, .vcom_mv = vcom_mv};
    return ESP_OK;
}

epd_tps_state_t epd_tps_state(const epd_tps_t *t)
{
    return t->state;
}

const epd_tps_stats_t *epd_tps_stats(const epd_tps_t *t)
{
    return &t->st;
}

static void wake_undo(epd_tps_t *t)
{
    line(t, EPD_TPS_LINE_WAKEUP, false);
    delay_us(t, EPD_TPS_SLEEP_MS * 1000u);
    t->state = EPD_TPS_SLEEP;
}

esp_err_t epd_tps_wake(epd_tps_t *t)
{
    if (t->state == EPD_TPS_FAILED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (awake(t)) {
        return ESP_OK;
    }
    esp_err_t err = line(t, EPD_TPS_LINE_WAKEUP, true);
    if (err != ESP_OK) {
        return err;
    }
    delay_us(t, EPD_TPS_WAKE_MS * 1000u);
    err = t->ops->probe(t->ops->ctx);
    uint8_t rev = 0;
    if (err == ESP_OK) {
        err = rd(t, EPD_TPS_REG_REVID, &rev);
    }
    if (err == ESP_OK && rev != EPD_TPS_REVID) {
        err = ESP_ERR_INVALID_RESPONSE;     // not the chip we validated: no power-up
    }
    // INT1/INT2 clear on read; keep the values for the log
    if (err == ESP_OK) {
        err = rd(t, EPD_TPS_REG_INT1, &t->st.int1);
    }
    if (err == ESP_OK) {
        err = rd(t, EPD_TPS_REG_INT2, &t->st.int2);
    }
    if (err == ESP_OK && (t->st.int1 & EPD_TPS_INT1_THERMAL)) {
        err = ESP_ERR_INVALID_STATE;        // thermal shutdown / hot: never ramp the rails
    }
    if (err != ESP_OK) {
        wake_undo(t);
        return err;
    }
    t->state = EPD_TPS_STANDBY;
    return ESP_OK;
}

static uint32_t ms_since(uint32_t polls)
{
    return polls * POLL_US / 1000u;
}

// One power-up attempt from SLEEP or STANDBY. The caller powers down on error.
static esp_err_t try_on(epd_tps_t *t)
{
    // Panel lines at their powered idle before anything else: with the
    // active-low start pulses (SPV, XSTL) held low the VEE charge pump failed
    // with VEE_UV on the device (plan 3 a2); the vendor firmware holds them high
    if (t->ops->bus_idle) {
        t->ops->bus_idle(t->ops->ctx, true);
    }
    t->ops->oe_low(t->ops->ctx);
    const bool was_standby = t->state == EPD_TPS_STANDBY;
    esp_err_t err = epd_tps_wake(t);
    // SLVSAQ8G 8.3.8: after a fault the rails stay off until INT1 and INT2
    // were read. wake() reads them coming out of SLEEP; between refreshes the
    // chip stays in STANDBY, so read (and check) them here as well.
    if (err == ESP_OK && was_standby) {
        err = rd(t, EPD_TPS_REG_INT1, &t->st.int1);
        if (err == ESP_OK) {
            err = rd(t, EPD_TPS_REG_INT2, &t->st.int2);
        }
        if (err == ESP_OK && (t->st.int1 & EPD_TPS_INT1_THERMAL)) {
            err = ESP_ERR_INVALID_STATE;
        }
    }
    if (err == ESP_OK) {
        err = wr_verify(t, EPD_TPS_REG_UPSEQ0, EPD_TPS_UPSEQ0);
    }
    if (err == ESP_OK) {
        err = wr_verify(t, EPD_TPS_REG_UPSEQ1, EPD_TPS_UPSEQ1);
    }
    if (err == ESP_OK) {
        err = write_vcom(t);    // in STANDBY: VCOM never drives the register default
    }
    if (err != ESP_OK) {
        return err;
    }
    t->pwrup = true;
    err = line(t, EPD_TPS_LINE_PWRUP, true);
    if (err != ESP_OK) {
        return err;
    }
    bool good = false;
    uint32_t polls = 0;
    for (;;) {
        err = t->ops->pg_pin(t->ops->ctx, &good);
        if (err != ESP_OK || good || polls >= EPD_TPS_PG_TIMEOUT_MS * 1000u / POLL_US) {
            break;
        }
        delay_us(t, POLL_US);
        polls++;
    }
    t->st.pg_ms = ms_since(polls);
    if (err == ESP_OK && !good) {
        err = ESP_ERR_TIMEOUT;
    }
    if (err != ESP_OK) {
        rd(t, EPD_TPS_REG_INT1, &t->st.int1);
        rd(t, EPD_TPS_REG_INT2, &t->st.int2);
        rd(t, EPD_TPS_REG_PG, &t->st.pg_reg);
        return err;
    }
    err = wr(t, EPD_TPS_REG_ENABLE, EPD_TPS_ENABLE_RAILS, true);
    if (err != ESP_OK) {
        return err;
    }
    polls = 0;
    for (;;) {
        err = rd(t, EPD_TPS_REG_PG, &t->st.pg_reg);
        if (err != ESP_OK || (t->st.pg_reg & EPD_TPS_PG_MASK) == EPD_TPS_PG_MASK ||
            polls >= EPD_TPS_PGREG_TIMEOUT_MS * 1000u / POLL_US) {
            break;
        }
        delay_us(t, POLL_US);
        polls++;
    }
    t->st.pgreg_ms = ms_since(polls);
    if (err == ESP_OK && (t->st.pg_reg & EPD_TPS_PG_MASK) != EPD_TPS_PG_MASK) {
        err = ESP_ERR_TIMEOUT;
    }
    if (err != ESP_OK) {
        rd(t, EPD_TPS_REG_INT1, &t->st.int1);
        rd(t, EPD_TPS_REG_INT2, &t->st.int2);
        return err;
    }
    err = line(t, EPD_TPS_LINE_VCOM_CTRL, true);    // VCOM last, after every rail is good
    if (err == ESP_OK) {
        t->state = EPD_TPS_ON;
    }
    return err;
}

esp_err_t epd_tps_hv_on(epd_tps_t *t)
{
    if (t->state == EPD_TPS_FAILED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (t->state == EPD_TPS_ON) {
        return ESP_OK;
    }
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (attempt) {
            delay_us(t, EPD_TPS_RETRY_MS * 1000u);
        }
        err = try_on(t);
        if (err == ESP_OK) {
            t->fails = 0;
            t->st.on_count++;
            return ESP_OK;
        }
        t->st.fail_count++;
        epd_tps_hv_off(t);
    }
    t->fails = 2;
    epd_tps_sleep(t);           // WAKEUP low too: the chip rests in SLEEP
    t->state = EPD_TPS_FAILED;  // until reboot: power-down stays allowed
    return err;
}

esp_err_t epd_tps_hv_off(epd_tps_t *t)
{
    t->ops->oe_low(t->ops->ctx);
    delay_us(t, EPD_TPS_OE_LEAD_US);
    esp_err_t err = line(t, EPD_TPS_LINE_VCOM_CTRL, false);
    const esp_err_t e2 = line(t, EPD_TPS_LINE_PWRUP, false);
    if (err == ESP_OK) {
        err = e2;
    }
    // PWR_GOOD means something only while the chip is awake (never while WAKEUP=0)
    if (t->pwrup && t->state != EPD_TPS_SLEEP) {
        bool good = true;
        uint32_t polls = 0;
        while (polls <= EPD_TPS_OFF_PG_MS * 1000u / POLL_US) {
            if (t->ops->pg_pin(t->ops->ctx, &good) != ESP_OK || !good) {
                break;
            }
            delay_us(t, POLL_US);
            polls++;
        }
        t->st.off_pg_ms = ms_since(polls);
        if (good && err == ESP_OK) {
            err = ESP_ERR_TIMEOUT;      // rails did not report down: logged by the caller
        }
    }
    t->pwrup = false;
    if (t->state == EPD_TPS_ON) {
        t->state = EPD_TPS_STANDBY;
    }
    return err;
}

esp_err_t epd_tps_sleep(epd_tps_t *t)
{
    esp_err_t err = epd_tps_hv_off(t);
    const bool was_sleep = t->state == EPD_TPS_SLEEP;
    const esp_err_t e2 = line(t, EPD_TPS_LINE_WAKEUP, false);
    if (!was_sleep) {
        delay_us(t, EPD_TPS_SLEEP_MS * 1000u);
    }
    if (t->ops->bus_idle) {
        t->ops->bus_idle(t->ops->ctx, false);   // panel 3.3 V is off in SLEEP: drive nothing high
    }
    if (t->state != EPD_TPS_FAILED) {
        t->state = EPD_TPS_SLEEP;
    }
    return err != ESP_OK ? err : e2;
}

esp_err_t epd_tps_set_vcom(epd_tps_t *t, uint16_t mv)
{
    if (mv < EPD_TPS_VCOM_MIN_MV || mv > EPD_TPS_VCOM_MAX_MV) {
        return ESP_ERR_INVALID_ARG;
    }
    if (t->state == EPD_TPS_ON) {
        return ESP_ERR_INVALID_STATE;   // only between refreshes, rails off
    }
    t->vcom_mv = mv;
    return t->state == EPD_TPS_STANDBY ? write_vcom(t) : ESP_OK;
}

esp_err_t epd_tps_temperature(epd_tps_t *t, int8_t *c)
{
    if (!awake(t)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = wr(t, EPD_TPS_REG_TMST1, EPD_TPS_TMST1_READ, false);
    uint8_t v = 0;
    for (int i = 0; err == ESP_OK && i < 20; i++) {
        err = rd(t, EPD_TPS_REG_TMST1, &v);
        if (err == ESP_OK && (v & EPD_TPS_TMST1_DONE)) {
            break;
        }
        delay_us(t, POLL_US);
    }
    if (err == ESP_OK && !(v & EPD_TPS_TMST1_DONE)) {
        return ESP_ERR_TIMEOUT;
    }
    if (err == ESP_OK) {
        err = rd(t, EPD_TPS_REG_TMST_VALUE, &v);
    }
    if (err == ESP_OK) {
        *c = (int8_t)v;
    }
    return err;
}

bool epd_tps_pg_ok(epd_tps_t *t)
{
    if (t->state != EPD_TPS_ON) {
        return false;
    }
    bool good = false;
    if (t->ops->pg_pin(t->ops->ctx, &good) == ESP_OK && good) {
        return true;
    }
    epd_tps_hv_off(t);
    return false;
}

esp_err_t epd_tps_reg_read(epd_tps_t *t, uint8_t reg, uint8_t *val)
{
    return awake(t) ? rd(t, reg, val) : ESP_ERR_INVALID_STATE;
}

esp_err_t epd_tps_reg_write(epd_tps_t *t, uint8_t reg, uint8_t val)
{
    if (!awake(t)) {
        return ESP_ERR_INVALID_STATE;
    }
    return wr(t, reg, val, false);
}
