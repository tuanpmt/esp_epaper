#pragma once

// LVGL 9.6 glue for e-paper (v3): LVGL renders L8 into one full-screen PSRAM
// buffer in DIRECT mode; flush_cb quantizes each rendered area straight into
// the caller's packed panel planes (rotation, dithering, invert in one pass),
// reports the panel rects of the frame with on_frame() and returns at once.
// The panel refresh itself runs elsewhere (an EPD task driving epd_sched).
// All functions run on the LVGL task.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "lvgl.h"
#include "epd_quant.h"
#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t hor_res, ver_res;              // logical size (3.97in: 480x800)
    epd_quant_t *quant;                     // owned by the caller, initialised, mode set
    // Called once per LVGL refresh, after its last area was quantized (LVGL task)
    void (*on_frame)(void *ctx, const epd_rect_t *panel_rects, uint8_t n);
    void *ctx;
    uint8_t *panel_plane0, *panel_plane1;   // packed planes the quantizer writes (plane1: GRAY2 only)
    size_t pan_stride;                      // bytes per panel row
    uint8_t align;                          // panel byte-axis alignment (8 for 1 bpp)
    uint8_t pad_x, pad_y;                   // invalidation padding for glyph edges (stock: 24, 12)
} epd_lvgl_cfg_t;

#define EPD_LVGL_MAX_RECTS 8

lv_display_t *epd_lvgl_create(const epd_lvgl_cfg_t *cfg);
// Invert or quantizer mode changed: quantize the whole L8 buffer again and
// report one full-screen frame.
void epd_lvgl_requantize_all(lv_display_t *d);

typedef struct {
    uint32_t render_us;     // last LVGL refresh, start to ready
    uint32_t quant_us;      // quantization inside the last refresh
    uint32_t last_area_px;  // logical pixels flushed in the last refresh
    uint32_t frames;
} epd_lvgl_stats_t;
void epd_lvgl_stats(lv_display_t *d, epd_lvgl_stats_t *out);

#ifdef __cplusplus
}
#endif
