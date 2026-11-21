#include <stdlib.h>
#include <string.h>
#include "epd_ssd1677.h"
#include "ssd1677_lut.h"

// Command set and sequences from the stock driver
// (CloudZao/Metalio-E-INK4 main/boards/metalio-e-ink-4/display/esp_lcd_panel_ssd1677.c, MIT).
#define CMD_DRIVER_OUTPUT 0x01
#define CMD_GATE_VOLTAGE 0x03
#define CMD_SOURCE_VOLTAGE 0x04
#define CMD_BOOSTER 0x0C
#define CMD_DEEP_SLEEP 0x10
#define CMD_DATA_ENTRY 0x11
#define CMD_SWRESET 0x12
#define CMD_TEMP_SENSOR 0x18
#define CMD_ACTIVATE 0x20
#define CMD_UPDATE_CTRL1 0x21
#define CMD_UPDATE_CTRL2 0x22
#define CMD_VCOM 0x2C
#define CMD_LUT 0x32
#define CMD_PING_PONG 0x37
#define CMD_BORDER 0x3C
#define CMD_POST_FULL 0x3F
#define CMD_RAM_X 0x44
#define CMD_RAM_Y 0x45
#define CMD_RAM_X_COUNTER 0x4E
#define CMD_RAM_Y_COUNTER 0x4F

#define CTRL2_FULL 0xC7
#define CTRL2_PARTIAL 0xCF
#define CTRL2_GRAY 0xCC
#define BORDER_DEFAULT 0x01
#define BORDER_PARTIAL 0x80
#define ENTRY_X_INC_Y_DEC 0x01

#define BUSY_POLL_LIMIT_MS 3000
#define ARM_SETTLE_MS 1         // let BUSY rise after 0x20 before arming the idle interrupt

// Partial LUT timing layout inside the 0x32 wave bytes
#define LUT_TIMING_AT 50        // after 5 VS rows x 10 groups
#define LUT_GROUPS 10
#define LUT_GROUP_BYTES 5       // TP A, B, C, D, RP
#define LUT_FR_AT 100
#define LUT_FR_BYTES 5

typedef enum { LUT_NONE, LUT_FAST, LUT_PARTIAL, LUT_GRAY } lut_t;

struct epd_ssd1677 {
    epd_ssd1677_cfg_t cfg;
    lut_t lut;
    bool border_partial;    // border waveform currently set for partial refreshes
    uint8_t partial_lut[SSD1677_LUT_SIZE];   // stock partial table with the tuned timing
    uint8_t *row_buf;       // one window, packed contiguously for a single data transfer
    size_t row_buf_size;
};

static esp_err_t cmd(epd_ssd1677_t *c, uint8_t op, const uint8_t *data, size_t len)
{
    return c->cfg.bus->cmd(c->cfg.bus, op, data, len);
}

static esp_err_t cmd1(epd_ssd1677_t *c, uint8_t op, uint8_t v)
{
    return cmd(c, op, &v, 1);
}

// Polled wait, used only during init and LUT loading (short)
static esp_err_t wait_idle(epd_ssd1677_t *c)
{
    for (uint32_t ms = 0; ms < BUSY_POLL_LIMIT_MS; ms++) {
        if (!c->cfg.bus->busy(c->cfg.bus)) {
            return ESP_OK;
        }
        c->cfg.bus->delay_ms(c->cfg.bus, 1);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t load_lut(epd_ssd1677_t *c, lut_t which)
{
    const uint8_t *t = which == LUT_FAST ? ssd1677_lut_fast : which == LUT_PARTIAL ? c->partial_lut
                                                                                   : ssd1677_lut_gray4;
    // Stock loader order: VCOM, gate, source, then the wave table
    esp_err_t err = cmd1(c, CMD_VCOM, t[109]);
    if (err == ESP_OK) {
        err = cmd1(c, CMD_GATE_VOLTAGE, t[105]);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_SOURCE_VOLTAGE, &t[106], 3);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_LUT, t, SSD1677_LUT_WAVE_BYTES);
    }
    if (err == ESP_OK) {
        err = wait_idle(c);
    }
    c->lut = err == ESP_OK ? which : LUT_NONE;
    return err;
}

static esp_err_t set_window(epd_ssd1677_t *c, epd_rect_t r)
{
    const uint16_t x0 = (uint16_t)r.x, x1 = (uint16_t)(r.x + r.w - 1);
    const uint16_t yrev = (uint16_t)(c->cfg.pan_h - r.y - r.h);
    const uint16_t ytop = (uint16_t)(yrev + r.h - 1);
    esp_err_t err = cmd1(c, CMD_DATA_ENTRY, ENTRY_X_INC_Y_DEC);
    if (err == ESP_OK) {
        err = cmd(c, CMD_RAM_X, (const uint8_t[]){x0 & 0xFF, x0 >> 8, x1 & 0xFF, x1 >> 8}, 4);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_RAM_Y, (const uint8_t[]){ytop & 0xFF, ytop >> 8, yrev & 0xFF, yrev >> 8}, 4);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_RAM_X_COUNTER, (const uint8_t[]){x0 & 0xFF, x0 >> 8}, 2);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_RAM_Y_COUNTER, (const uint8_t[]){ytop & 0xFF, ytop >> 8}, 2);
    }
    return err;
}

static esp_err_t init_sequence(epd_ssd1677_t *c)
{
    c->cfg.bus->reset(c->cfg.bus);
    c->lut = LUT_NONE;
    c->border_partial = false;
    esp_err_t err = wait_idle(c);
    if (err == ESP_OK) {
        err = cmd(c, CMD_SWRESET, NULL, 0);
    }
    if (err == ESP_OK) {
        c->cfg.bus->delay_ms(c->cfg.bus, 10);
        err = wait_idle(c);
    }
    const uint16_t mux = (uint16_t)(c->cfg.pan_h - 1);
    if (err == ESP_OK) {
        err = cmd1(c, CMD_TEMP_SENSOR, 0x80);   // internal sensor
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_PING_PONG, (const uint8_t[]){0, 0, 0, 0, 0, 0x40, 0, 0, 0, 0}, 10);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_BOOSTER, (const uint8_t[]){0xAE, 0xC7, 0xC3, 0xC0, 0x80}, 5);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_DRIVER_OUTPUT, (const uint8_t[]){mux & 0xFF, mux >> 8, 0x02}, 3);
    }
    if (err == ESP_OK) {
        err = cmd1(c, CMD_BORDER, BORDER_DEFAULT);
    }
    if (err == ESP_OK) {
        err = load_lut(c, LUT_FAST);
    }
    if (err == ESP_OK) {
        err = set_window(c, (epd_rect_t){0, 0, (int16_t)c->cfg.pan_w, (int16_t)c->cfg.pan_h});
    }
    return err;
}

esp_err_t epd_ssd1677_init(const epd_ssd1677_cfg_t *cfg, epd_ssd1677_t **out)
{
    if (!cfg || !cfg->bus || !out || cfg->pan_w % 8 || !cfg->pan_w || !cfg->pan_h) {
        return ESP_ERR_INVALID_ARG;
    }
    epd_ssd1677_t *c = calloc(1, sizeof(*c));
    if (!c) {
        return ESP_ERR_NO_MEM;
    }
    c->cfg = *cfg;
    memcpy(c->partial_lut, ssd1677_lut_partial, SSD1677_LUT_SIZE);
    const esp_err_t err = init_sequence(c);
    if (err != ESP_OK) {
        free(c);
        return err;
    }
    *out = c;
    return ESP_OK;
}

void epd_ssd1677_free(epd_ssd1677_t *c)
{
    if (c) {
        free(c->row_buf);
        free(c);
    }
}

static bool window_ok(const epd_ssd1677_t *c, epd_rect_t r)
{
    return r.w > 0 && r.h > 0 && r.x >= 0 && r.y >= 0 && r.x % 8 == 0 && r.w % 8 == 0 &&
           r.x + r.w <= c->cfg.pan_w && r.y + r.h <= c->cfg.pan_h;
}

esp_err_t epd_ssd1677_write(epd_ssd1677_t *c, uint8_t ram, epd_rect_t r, const uint8_t *plane, size_t stride)
{
    if (!c || !plane || (ram != EPD_SSD1677_RAM_NEW && ram != EPD_SSD1677_RAM_OLD) || !window_ok(c, r) ||
        stride < (size_t)(r.x + r.w) / 8) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t row = (size_t)r.w / 8;
    const size_t need = row * (size_t)r.h;
    if (need > c->row_buf_size) {
        uint8_t *b = realloc(c->row_buf, need);
        if (!b) {
            return ESP_ERR_NO_MEM;
        }
        c->row_buf = b;
        c->row_buf_size = need;
    }
    // Rows top to bottom: with Y decrementing from the reversed window top,
    // the first row sent lands on panel row r.y
    for (int16_t y = 0; y < r.h; y++) {
        memcpy(c->row_buf + (size_t)y * row, plane + (size_t)(r.y + y) * stride + (size_t)r.x / 8, row);
    }
    esp_err_t err = set_window(c, r);
    if (err == ESP_OK) {
        err = cmd(c, ram, NULL, 0);
    }
    if (err == ESP_OK) {
        err = c->cfg.bus->data(c->cfg.bus, c->row_buf, need);
    }
    return err;
}

esp_err_t epd_ssd1677_start(epd_ssd1677_t *c, epd_mode_t mode, epd_busy_cb_t done, void *ctx)
{
    if (!c || !done) {
        return ESP_ERR_INVALID_ARG;
    }
    const lut_t want = mode == EPD_M_FULL_CLEAN ? LUT_FAST : mode == EPD_M_GRAY ? LUT_GRAY : LUT_PARTIAL;
    esp_err_t err = ESP_OK;
    if (c->lut != want) {
        err = load_lut(c, want);
    }
    const bool partial = want == LUT_PARTIAL;
    if (err == ESP_OK && partial != c->border_partial) {
        // Stock "prepare for partial": border floating during partials, default otherwise
        err = cmd1(c, CMD_BORDER, partial ? BORDER_PARTIAL : BORDER_DEFAULT);
        c->border_partial = partial && err == ESP_OK;
    }
    if (err == ESP_OK && want == LUT_GRAY) {
        err = cmd1(c, CMD_UPDATE_CTRL1, 0x00);   // use the RED plane (no bypass) for 4 gray
    }
    if (err == ESP_OK) {
        err = cmd1(c, CMD_UPDATE_CTRL2, want == LUT_FAST ? CTRL2_FULL : want == LUT_GRAY ? CTRL2_GRAY : CTRL2_PARTIAL);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_ACTIVATE, NULL, 0);
    }
    if (err == ESP_OK) {
        c->cfg.bus->delay_ms(c->cfg.bus, ARM_SETTLE_MS);
        err = c->cfg.bus->arm_busy_irq(c->cfg.bus, done, ctx);
    }
    return err;
}

esp_err_t epd_ssd1677_after(epd_ssd1677_t *c, epd_mode_t mode, epd_rect_t r, const uint8_t *bw_plane, size_t stride)
{
    if (!c) {
        return ESP_ERR_INVALID_ARG;
    }
    switch (mode) {
    case EPD_M_FULL_CLEAN:
        return cmd1(c, CMD_POST_FULL, 0x01);    // stock non-optimised path: after BUSY of a full refresh
    case EPD_M_PARTIAL:
    case EPD_M_FAST:
        return c->cfg.auto_old_plane ? ESP_OK : epd_ssd1677_write(c, EPD_SSD1677_RAM_OLD, r, bw_plane, stride);
    case EPD_M_GRAY: {
        esp_err_t err = epd_ssd1677_write(c, EPD_SSD1677_RAM_NEW, r, bw_plane, stride);
        return err == ESP_OK ? epd_ssd1677_write(c, EPD_SSD1677_RAM_OLD, r, bw_plane, stride) : err;
    }
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t epd_ssd1677_temperature(epd_ssd1677_t *c, int8_t *deg_c)
{
    (void)c;
    (void)deg_c;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t epd_ssd1677_deep_sleep(epd_ssd1677_t *c)
{
    if (!c) {
        return ESP_ERR_INVALID_ARG;
    }
    c->lut = LUT_NONE;
    return cmd1(c, CMD_DEEP_SLEEP, 0x01);
}

esp_err_t epd_ssd1677_reinit(epd_ssd1677_t *c)
{
    return c ? init_sequence(c) : ESP_ERR_INVALID_ARG;
}

void epd_ssd1677_partial_timing(const epd_ssd1677_t *c, epd_ssd1677_timing_t *out)
{
    if (!c || !out) {
        return;
    }
    for (int g = 0; g < LUT_GROUPS; g++) {
        const uint8_t *grp = &c->partial_lut[LUT_TIMING_AT + g * LUT_GROUP_BYTES];
        memcpy(out->tp[g], grp, 4);
        out->rp[g] = grp[4];
    }
    memcpy(out->fr, &c->partial_lut[LUT_FR_AT], LUT_FR_BYTES);
}

esp_err_t epd_ssd1677_set_partial_timing(epd_ssd1677_t *c, const epd_ssd1677_timing_t *t)
{
    if (!c) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t lut[SSD1677_LUT_SIZE];
    memcpy(lut, ssd1677_lut_partial, SSD1677_LUT_SIZE);
    if (t) {
        for (int g = 0; g < LUT_GROUPS; g++) {
            uint8_t *grp = &lut[LUT_TIMING_AT + g * LUT_GROUP_BYTES];
            for (int i = 0; i < 4; i++) {
                if (t->tp[g][i] > grp[i]) {
                    return ESP_ERR_INVALID_ARG;
                }
                grp[i] = t->tp[g][i];
            }
            if (t->rp[g] > grp[4]) {
                return ESP_ERR_INVALID_ARG;
            }
            grp[4] = t->rp[g];
        }
        memcpy(&lut[LUT_FR_AT], t->fr, LUT_FR_BYTES);
    }
    memcpy(c->partial_lut, lut, SSD1677_LUT_SIZE);
    if (c->lut == LUT_PARTIAL) {
        c->lut = LUT_NONE;  // reload at the next partial/fast start
    }
    return ESP_OK;
}
