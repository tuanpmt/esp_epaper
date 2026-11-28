#include "epd_bus_rec.h"

#include <stdlib.h>
#include <string.h>

static epd_rec_t *rec_push(epd_bus_rec_t *r, epd_rec_type_t type)
{
    if (r->n == r->cap) {
        size_t cap = r->cap ? r->cap * 2 : 64;
        epd_rec_t *log = realloc(r->log, cap * sizeof *log);
        if (!log) {
            abort();
        }
        r->log = log;
        r->cap = cap;
    }
    epd_rec_t *e = &r->log[r->n++];
    memset(e, 0, sizeof *e);
    e->type = type;
    return e;
}

static esp_err_t rec_copy(epd_rec_t *e, const uint8_t *data, size_t len)
{
    if (len == 0) {
        return ESP_OK;
    }
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    e->bytes = malloc(len);
    if (!e->bytes) {
        abort();
    }
    memcpy(e->bytes, data, len);
    e->len = len;
    return ESP_OK;
}

static esp_err_t rec_cmd(epd_bus_t *b, uint8_t cmd, const uint8_t *data, size_t len)
{
    epd_rec_t *e = rec_push((epd_bus_rec_t *)b, EPD_REC_CMD);
    e->cmd = cmd;
    return rec_copy(e, data, len);
}

static esp_err_t rec_data(epd_bus_t *b, const uint8_t *data, size_t len)
{
    return rec_copy(rec_push((epd_bus_rec_t *)b, EPD_REC_DATA), data, len);
}

static bool rec_busy(epd_bus_t *b)
{
    epd_bus_rec_t *r = (epd_bus_rec_t *)b;
    if (r->low_reads > 0) {
        r->low_reads--;
        return false;
    }
    return r->busy_level;
}

static esp_err_t rec_arm(epd_bus_t *b, epd_busy_cb_t cb, void *ctx)
{
    epd_bus_rec_t *r = (epd_bus_rec_t *)b;
    if (!cb) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!r->reset_seen) {
        return ESP_ERR_INVALID_STATE;
    }
    rec_push(r, EPD_REC_ARM);
    if (!r->busy_level) {   // already idle: run now, like the device
        cb(ctx);
        return ESP_OK;
    }
    r->cb = cb;
    r->cb_ctx = ctx;
    return ESP_OK;
}

static void rec_reset(epd_bus_t *b)
{
    epd_bus_rec_t *r = (epd_bus_rec_t *)b;
    rec_push(r, EPD_REC_RESET);
    r->reset_seen = true;
}

static void rec_delay(epd_bus_t *b, uint32_t ms)
{
    epd_bus_rec_t *r = (epd_bus_rec_t *)b;
    rec_push(r, EPD_REC_DELAY)->ms = ms;
    r->clock_ms += r->tick_ms > ms ? r->tick_ms : ms;   // a tick-rounded sleep when tick_ms is set
}

static uint32_t rec_now(epd_bus_t *b)
{
    return ((epd_bus_rec_t *)b)->clock_ms;
}

void epd_bus_rec_init(epd_bus_rec_t *r)
{
    memset(r, 0, sizeof *r);
    r->bus.cmd = rec_cmd;
    r->bus.data = rec_data;
    r->bus.busy = rec_busy;
    r->bus.arm_busy_irq = rec_arm;
    r->bus.reset = rec_reset;
    r->bus.delay_ms = rec_delay;
    r->bus.now_ms = rec_now;
}

void epd_bus_rec_clear(epd_bus_rec_t *r)
{
    for (size_t i = 0; i < r->n; i++) {
        free(r->log[i].bytes);
    }
    r->n = 0;
}

void epd_bus_rec_free(epd_bus_rec_t *r)
{
    epd_bus_rec_clear(r);
    free(r->log);
    r->log = NULL;
    r->cap = 0;
}

void epd_bus_rec_set_busy(epd_bus_rec_t *r, bool busy)
{
    r->busy_level = busy;
}

bool epd_bus_rec_fire_busy(epd_bus_rec_t *r)
{
    r->busy_level = false;
    epd_busy_cb_t cb = r->cb;
    if (!cb) {
        return false;
    }
    r->cb = NULL;   // one-shot
    cb(r->cb_ctx);
    return true;
}

int epd_bus_rec_find_cmd(const epd_bus_rec_t *r, uint8_t cmd, size_t from)
{
    for (size_t i = from; i < r->n; i++) {
        if (r->log[i].type == EPD_REC_CMD && r->log[i].cmd == cmd) {
            return (int)i;
        }
    }
    return -1;
}
