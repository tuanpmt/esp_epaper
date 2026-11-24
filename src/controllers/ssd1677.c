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
#define ARM_RISE_MAX_MS 20      // poll for BUSY to rise after 0x20 before arming the idle interrupt

// Partial LUT timing layout inside the 0x32 wave bytes
#define LUT_TIMING_AT 50        // after 5 VS rows x 10 groups
#define LUT_GROUPS 10
#define LUT_GROUP_BYTES 5       // TP A, B, C, D, RP
#define LUT_FR_AT 100
#define LUT_FR_BYTES 5

// LUT_FULL: the stock "fast update" table of full cleans (0xC7); LUT_QUICK:
// the fast (interaction) copy of the partial table; LUT_TRANS: the copy used
// by screen changes (EPD_M_TRANSITION)
typedef enum { LUT_NONE, LUT_FULL, LUT_PARTIAL, LUT_QUICK, LUT_GRAY, LUT_TRANS } lut_t;

struct epd_ssd1677 {
    epd_ssd1677_cfg_t cfg;
    lut_t lut;
    bool border_partial;    // border waveform currently set for partial refreshes
    uint8_t partial_lut[SSD1677_LUT_SIZE];   // stock partial table with the tuned timing
    uint8_t quick_lut[SSD1677_LUT_SIZE];     // same for EPD_M_FAST
    uint8_t trans_lut[SSD1677_LUT_SIZE];     // same for EPD_M_TRANSITION
    uint8_t table_id[EPD_SSD1677_ROLE_COUNT];   // candidate table per role, 0 = set by timing
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

static const uint8_t *lut_table(const epd_ssd1677_t *c, lut_t which)
{
    switch (which) {
    case LUT_FULL:
        return ssd1677_lut_fast;
    case LUT_PARTIAL:
        return c->partial_lut;
    case LUT_QUICK:
        return c->quick_lut;
    case LUT_TRANS:
        return c->trans_lut;
    default:
        return ssd1677_lut_gray4;
    }
}

static bool partial_kind(lut_t which)
{
    return which == LUT_PARTIAL || which == LUT_QUICK || which == LUT_TRANS;
}

static esp_err_t load_lut(epd_ssd1677_t *c, lut_t which)
{
    const uint8_t *t = lut_table(c, which);
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
        err = load_lut(c, LUT_FULL);
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
    memcpy(c->quick_lut, ssd1677_lut_partial, SSD1677_LUT_SIZE);
    memcpy(c->trans_lut, ssd1677_lut_partial, SSD1677_LUT_SIZE);
    epd_ssd1677_use_table(c, EPD_SSD1677_ROLE_PAGE, ssd1677_lut_pick_partial);
    epd_ssd1677_use_table(c, EPD_SSD1677_ROLE_SCREEN, ssd1677_lut_pick_partial);
    epd_ssd1677_use_table(c, EPD_SSD1677_ROLE_BUTTON, ssd1677_lut_pick_fast);
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
    const lut_t want = mode == EPD_M_FULL_CLEAN ? LUT_FULL
                       : mode == EPD_M_GRAY     ? LUT_GRAY
                       : mode == EPD_M_FAST       ? LUT_QUICK
                       : mode == EPD_M_TRANSITION ? LUT_TRANS
                                                  : LUT_PARTIAL;
    esp_err_t err = ESP_OK;
    if (c->lut != want) {
        if (partial_kind(c->lut) && partial_kind(want) &&
            memcmp(lut_table(c, c->lut), lut_table(c, want), SSD1677_LUT_SIZE) == 0) {
            c->lut = want;      // the controller already holds the same bytes
        } else {
            err = load_lut(c, want);
        }
    }
    const bool partial = partial_kind(want);
    if (err == ESP_OK && partial != c->border_partial) {
        // Stock "prepare for partial": border floating during partials, default otherwise
        err = cmd1(c, CMD_BORDER, partial ? BORDER_PARTIAL : BORDER_DEFAULT);
        c->border_partial = partial && err == ESP_OK;
    }
    if (err == ESP_OK && want == LUT_GRAY) {
        err = cmd1(c, CMD_UPDATE_CTRL1, 0x00);   // use the RED plane (no bypass) for 4 gray
    }
    if (err == ESP_OK) {
        err = cmd1(c, CMD_UPDATE_CTRL2, want == LUT_FULL ? CTRL2_FULL : want == LUT_GRAY ? CTRL2_GRAY : CTRL2_PARTIAL);
    }
    if (err == ESP_OK) {
        err = cmd(c, CMD_ACTIVATE, NULL, 0);
    }
    if (err == ESP_OK) {
        // BUSY rises shortly after 0x20; arming the idle interrupt before it
        // rises would report the refresh done at once. If it never rises the
        // refresh is armed anyway (the callback fires now, the run looks instant).
        for (uint32_t ms = 0; ms < ARM_RISE_MAX_MS && !c->cfg.bus->busy(c->cfg.bus); ms++) {
            c->cfg.bus->delay_ms(c->cfg.bus, 1);
        }
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
    case EPD_M_TRANSITION:
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

static void read_timing(const uint8_t *lut, epd_ssd1677_timing_t *out)
{
    for (int g = 0; g < LUT_GROUPS; g++) {
        const uint8_t *grp = &lut[LUT_TIMING_AT + g * LUT_GROUP_BYTES];
        memcpy(out->tp[g], grp, 4);
        out->rp[g] = grp[4];
    }
    memcpy(out->fr, &lut[LUT_FR_AT], LUT_FR_BYTES);
}

// Stock partial table with t's timing into dst; NULL t = stock. Refused (dst
// untouched) when a TP or RP is longer than stock.
static esp_err_t patch_timing(uint8_t *dst, const epd_ssd1677_timing_t *t)
{
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
    memcpy(dst, lut, SSD1677_LUT_SIZE);
    return ESP_OK;
}

static esp_err_t set_timing(epd_ssd1677_t *c, lut_t which, const epd_ssd1677_timing_t *t)
{
    if (!c) {
        return ESP_ERR_INVALID_ARG;
    }
    const esp_err_t err = patch_timing(which == LUT_QUICK ? c->quick_lut : c->partial_lut, t);
    if (err == ESP_OK && which == LUT_PARTIAL) {
        memcpy(c->trans_lut, c->partial_lut, SSD1677_LUT_SIZE);    // screen changes follow page turns
    }
    if (err == ESP_OK) {
        if (which == LUT_QUICK) {
            c->table_id[EPD_SSD1677_ROLE_BUTTON] = 0;
        } else {
            c->table_id[EPD_SSD1677_ROLE_PAGE] = c->table_id[EPD_SSD1677_ROLE_SCREEN] = 0;
        }
        if (partial_kind(c->lut)) {
            c->lut = LUT_NONE;  // reload at the next start that uses it
        }
    }
    return err;
}

void epd_ssd1677_partial_timing(const epd_ssd1677_t *c, epd_ssd1677_timing_t *out)
{
    if (c && out) {
        read_timing(c->partial_lut, out);
    }
}

esp_err_t epd_ssd1677_set_partial_timing(epd_ssd1677_t *c, const epd_ssd1677_timing_t *t)
{
    return set_timing(c, LUT_PARTIAL, t);
}

void epd_ssd1677_fast_timing(const epd_ssd1677_t *c, epd_ssd1677_timing_t *out)
{
    if (c && out) {
        read_timing(c->quick_lut, out);
    }
}

esp_err_t epd_ssd1677_set_fast_timing(epd_ssd1677_t *c, const epd_ssd1677_timing_t *t)
{
    return set_timing(c, LUT_QUICK, t);
}

esp_err_t epd_ssd1677_lut_candidate(uint8_t id, epd_ssd1677_timing_t *out)
{
    if (!out || id < 1 || id > EPD_SSD1677_LUT_CANDIDATES) {
        return ESP_ERR_INVALID_ARG;
    }
    read_timing(ssd1677_lut_partial, out);
    if (id >= 2) {
        memset(out->fr, id <= 3 ? 0x44 : 0x55, sizeof(out->fr));
    }
    if (id == 3 || id == 5) {
        out->tp[0][0] = 6;
    }
    return ESP_OK;
}

// ---- Candidate tables (plan 9 Task 12) ----
//
// Each candidate is the stock partial table with its timing (FR, TP A of
// group 0) and, for 8..12, VS rows of groups 0 and 1 changed. Row r of group
// g is byte r * 10 + g, two bits per phase A..D (A in bits 7..6): 00 VSS,
// 01 VSH1 (towards black), 10 VSL (towards white), 11 VSH2. Rows: 0 black
// stays black, 1 black to white, 2 white to black, 3 white stays white,
// 4 VCOM. Stock: group 0 drives the changing pixels for 13 frames (row 1
// VSL, row 2 VSH1); group 1 gives the black pixels that stay black 4 frames
// of VSH2. Voltages (bytes 105..111) never change.
//
// DC balance per candidate (VSH1 +15 V, VSL -15 V, 0x41 / 0x32):
//   1..7   as stock: a changing pixel gets one polarity per change, so going
//          white -> black -> white nets zero; static black gets 4 frames of
//          VSH2 per run (stock behaviour), static white nothing.
//   8..10, 13, 14  "deepen": group 1 repeats the change's polarity for 4 frames on
//          both directions (VSL to white, VSH1 to black), symmetric, so a
//          pixel going back and forth still nets zero.
//   11     text: 8 plus 4 frames of VSL on white that stays white each run.
//          Static white pixels accumulate negative drive until the next full
//          clean (0xC7, balanced) of the ghost budget; mirrors the stock VSH2
//          on static black. Do not ship without a ghost budget.
//   12     A2: group 0 drives every pixel 8 frames to its level (static
//          black VSH1, static white VSL), plus the stock VSH2 on static black.
//          Static pixels accumulate one-sided drive per run; same caveat as 11.
#define VS_ROW(r, g) ((r) * LUT_GROUPS + (g))
#define PH_A_VSH1 0x40
#define PH_A_VSL 0x80

static const char *const s_table_names[EPD_SSD1677_LUT_TABLES + 1] = {
    "",
    "stock FR 0x22",
    "FR 0x44",
    "FR 0x44 + TP A6",
    "FR 0x55 (page default)",
    "FR 0x55 + TP A6 (button default)",
    "FR 0x44 + TP A8",
    "FR 0x55 + TP A8",
    "FR 0x55 + deepen",
    "FR 0x55 + TP A8 + deepen",
    "FR 0x44 + deepen",
    "text: FR 0x55 + deepen + white re-drive",
    "A2: FR 0x55, every pixel driven 8 frames",
    "erase: FR 0x33 + deepen",
    "erase: stock FR 0x22 + deepen",
};

const char *epd_ssd1677_table_name(uint8_t id)
{
    return id >= 1 && id <= EPD_SSD1677_LUT_TABLES ? s_table_names[id] : NULL;
}

esp_err_t epd_ssd1677_table(uint8_t id, uint8_t out[EPD_SSD1677_LUT_SIZE])
{
    if (!out || id < 1 || id > EPD_SSD1677_LUT_TABLES) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t *t = out;
    memcpy(t, ssd1677_lut_partial, SSD1677_LUT_SIZE);
    uint8_t fr = 0x22, tp_a0 = 0x0D;
    bool deepen = false, white = false, a2 = false;
    switch (id) {
    case 2: fr = 0x44; break;
    case 3: fr = 0x44; tp_a0 = 6; break;
    case 4: fr = 0x55; break;
    case 5: fr = 0x55; tp_a0 = 6; break;
    case 6: fr = 0x44; tp_a0 = 8; break;
    case 7: fr = 0x55; tp_a0 = 8; break;
    case 8: fr = 0x55; deepen = true; break;
    case 9: fr = 0x55; tp_a0 = 8; deepen = true; break;
    case 10: fr = 0x44; deepen = true; break;
    case 11: fr = 0x55; deepen = true; white = true; break;
    case 12: fr = 0x55; tp_a0 = 8; a2 = true; break;
    case 13: fr = 0x33; deepen = true; break;
    case 14: deepen = true; break;
    default: break;
    }
    memset(&t[LUT_FR_AT], fr, LUT_FR_BYTES);
    t[LUT_TIMING_AT] = tp_a0;
    if (deepen) {
        // Group 1 (4 frames, after the main drive): the changed pixels get
        // their polarity again, white and black alike
        t[VS_ROW(1, 1)] = PH_A_VSL;
        t[VS_ROW(2, 1)] = PH_A_VSH1;
    }
    if (white) {
        t[VS_ROW(3, 1)] = PH_A_VSL;     // white that stays white: 4 frames towards white
    }
    if (a2) {
        t[VS_ROW(0, 0)] = PH_A_VSH1;    // every pixel to its level, changed or not
        t[VS_ROW(3, 0)] = PH_A_VSL;
    }
    return epd_ssd1677_table_check(t);
}

// Every pixel then gets at most the 17 frames stock drives, none longer than
// a stock frame, at stock voltages
esp_err_t epd_ssd1677_table_check(const uint8_t *t)
{
    const uint8_t *s = ssd1677_lut_partial;
    if (!t || memcmp(&t[SSD1677_LUT_WAVE_BYTES], &s[SSD1677_LUT_WAVE_BYTES],
                     SSD1677_LUT_SIZE - SSD1677_LUT_WAVE_BYTES) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int g = 0; g < LUT_GROUPS; g++) {
        if (t[VS_ROW(4, g)] != s[VS_ROW(4, g)]) {
            return ESP_ERR_INVALID_ARG;
        }
        const uint8_t *tg = &t[LUT_TIMING_AT + g * LUT_GROUP_BYTES];
        const uint8_t *sg = &s[LUT_TIMING_AT + g * LUT_GROUP_BYTES];
        int frames = 0, stock = 0;
        for (int i = 0; i < 4; i++) {
            frames += tg[i];
            stock += sg[i];
        }
        if (frames > stock || tg[4] > sg[4]) {
            return ESP_ERR_INVALID_ARG;
        }
        for (int r = 0; g >= 2 && r < 4; r++) {
            if (t[VS_ROW(r, g)] != 0) {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    for (int i = 0; i < LUT_FR_BYTES; i++) {
        const uint8_t f = t[LUT_FR_AT + i], sf = s[LUT_FR_AT + i];
        if ((f >> 4) < (sf >> 4) || (f & 0x0F) < (sf & 0x0F)) {
            return ESP_ERR_INVALID_ARG;
        }
    }
    return ESP_OK;
}

esp_err_t epd_ssd1677_use_table(epd_ssd1677_t *c, epd_ssd1677_role_t role, uint8_t id)
{
    if (!c || role >= EPD_SSD1677_ROLE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t t[SSD1677_LUT_SIZE];
    const esp_err_t err = epd_ssd1677_table(id, t);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t *dst = role == EPD_SSD1677_ROLE_PAGE     ? c->partial_lut
                   : role == EPD_SSD1677_ROLE_SCREEN ? c->trans_lut
                                                     : c->quick_lut;
    memcpy(dst, t, SSD1677_LUT_SIZE);
    c->table_id[role] = id;
    if (partial_kind(c->lut)) {
        c->lut = LUT_NONE;  // reload at the next partial start
    }
    return ESP_OK;
}

uint8_t epd_ssd1677_table_in_use(const epd_ssd1677_t *c, epd_ssd1677_role_t role)
{
    return c && role < EPD_SSD1677_ROLE_COUNT ? c->table_id[role] : 0;
}
