#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "epd_lvgl.h"
#include "epd_round.h"

static const char *TAG = "epd_lvgl";

typedef struct {
    epd_lvgl_cfg_t cfg;
    uint8_t *l8;                            // full-screen L8 render buffer (PSRAM)
    epd_rect_t rects[EPD_LVGL_MAX_RECTS];   // panel rects of the refresh in progress
    uint8_t n;
    int64_t refr_start_us;
    uint32_t quant_acc_us, area_acc_px;
    epd_lvgl_stats_t stats;
} glue_t;

static epd_rect_t bbox(epd_rect_t a, epd_rect_t b)
{
    const int32_t x0 = a.x < b.x ? a.x : b.x, y0 = a.y < b.y ? a.y : b.y;
    const int32_t x1 = (a.x + a.w) > (b.x + b.w) ? a.x + a.w : b.x + b.w;
    const int32_t y1 = (a.y + a.h) > (b.y + b.h) ? a.y + a.h : b.y + b.h;
    return (epd_rect_t){(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0), (int16_t)(y1 - y0)};
}

static void add_rect(glue_t *g, epd_rect_t r)
{
    if (r.w <= 0 || r.h <= 0) {
        return;
    }
    if (g->n < EPD_LVGL_MAX_RECTS) {
        g->rects[g->n++] = r;
    } else {
        g->rects[EPD_LVGL_MAX_RECTS - 1] = bbox(g->rects[EPD_LVGL_MAX_RECTS - 1], r);   // the scheduler merges further
    }
}

static void quantize_area(glue_t *g, const lv_area_t *area)
{
    const epd_rect_t lr = {(int16_t)area->x1, (int16_t)area->y1, (int16_t)lv_area_get_width(area),
                           (int16_t)lv_area_get_height(area)};
    const int64_t t0 = esp_timer_get_time();
    epd_quant_rect(g->cfg.quant, g->l8, g->cfg.hor_res, lr, g->cfg.panel_plane0, g->cfg.panel_plane1,
                   g->cfg.pan_stride);
    g->quant_acc_us += (uint32_t)(esp_timer_get_time() - t0);
    g->area_acc_px += (uint32_t)lr.w * (uint32_t)lr.h;
    add_rect(g, epd_quant_map_rect(g->cfg.quant, lr));
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    (void)px_map;   // DIRECT mode: px_map is the buffer base, the same as g->l8
    glue_t *g = lv_display_get_driver_data(disp);
    quantize_area(g, area);
    if (lv_display_flush_is_last(disp)) {
        g->stats.quant_us = g->quant_acc_us;
        g->stats.last_area_px = g->area_acc_px;
        g->stats.frames++;
        if (g->n && g->cfg.on_frame) {
            g->cfg.on_frame(g->cfg.ctx, g->rects, g->n);
        }
        g->n = 0;
        g->quant_acc_us = g->area_acc_px = 0;
    }
    lv_display_flush_ready(disp);
}

static void event_cb(lv_event_t *e)
{
    lv_display_t *disp = lv_event_get_target(e);
    glue_t *g = lv_display_get_driver_data(disp);
    switch (lv_event_get_code(e)) {
    case LV_EVENT_INVALIDATE_AREA: {
        lv_area_t *a = lv_event_get_param(e);
        const epd_rot_t rot = g->cfg.quant->rot;
        const epd_rect_t r = epd_round_rect(
            (epd_rect_t){(int16_t)a->x1, (int16_t)a->y1, (int16_t)lv_area_get_width(a), (int16_t)lv_area_get_height(a)},
            g->cfg.hor_res, g->cfg.ver_res, rot, g->cfg.align, g->cfg.pad_x, g->cfg.pad_y);
        if (r.w > 0 && r.h > 0) {
            a->x1 = r.x;
            a->y1 = r.y;
            a->x2 = r.x + r.w - 1;
            a->y2 = r.y + r.h - 1;
        }
        break;
    }
    case LV_EVENT_REFR_START:
        g->refr_start_us = esp_timer_get_time();
        break;
    case LV_EVENT_REFR_READY:
        g->stats.render_us = (uint32_t)(esp_timer_get_time() - g->refr_start_us);
        break;
    default:
        break;
    }
}

lv_display_t *epd_lvgl_create(const epd_lvgl_cfg_t *cfg)
{
    if (!cfg || !cfg->quant || !cfg->panel_plane0 || !cfg->hor_res || !cfg->ver_res) {
        return NULL;
    }
    glue_t *g = calloc(1, sizeof(*g));
    const size_t size = (size_t)cfg->hor_res * cfg->ver_res;
    uint8_t *l8 = heap_caps_aligned_alloc(4, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g || !l8) {
        ESP_LOGE(TAG, "no memory for the %u B L8 buffer", (unsigned)size);
        free(g);
        heap_caps_free(l8);
        return NULL;
    }
    memset(l8, 0xFF, size);     // white paper until LVGL draws
    g->cfg = *cfg;
    g->l8 = l8;
    lv_display_t *disp = lv_display_create(cfg->hor_res, cfg->ver_res);
    if (!disp) {
        free(g);
        heap_caps_free(l8);
        return NULL;
    }
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_L8);
    lv_display_set_buffers(disp, l8, NULL, size, LV_DISPLAY_RENDER_MODE_DIRECT);
    lv_display_set_driver_data(disp, g);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_add_event_cb(disp, event_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    lv_display_add_event_cb(disp, event_cb, LV_EVENT_REFR_START, NULL);
    lv_display_add_event_cb(disp, event_cb, LV_EVENT_REFR_READY, NULL);
    ESP_LOGI(TAG, "L8 DIRECT %ux%u, buffer %u B in PSRAM", cfg->hor_res, cfg->ver_res, (unsigned)size);
    return disp;
}

void epd_lvgl_requantize_all(lv_display_t *d)
{
    glue_t *g = lv_display_get_driver_data(d);
    if (!g) {
        return;
    }
    const lv_area_t all = {0, 0, g->cfg.hor_res - 1, g->cfg.ver_res - 1};
    g->n = 0;
    g->quant_acc_us = g->area_acc_px = 0;
    quantize_area(g, &all);
    if (g->cfg.on_frame) {
        g->cfg.on_frame(g->cfg.ctx, g->rects, g->n);
    }
    g->n = 0;
}

void epd_lvgl_stats(lv_display_t *d, epd_lvgl_stats_t *out)
{
    glue_t *g = lv_display_get_driver_data(d);
    if (g && out) {
        *out = g->stats;
    }
}
