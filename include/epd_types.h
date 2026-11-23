#pragma once

// Plain types shared by the panel-independent v3 modules (quantizer, dirty
// set, scheduler) and the controllers. No ESP-IDF dependency: host-testable.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// A rectangle in pixels; w or h <= 0 means empty
typedef struct {
    int16_t x, y, w, h;
} epd_rect_t;

// Logical (LVGL) to panel orientation
typedef enum { EPD_ROT_0, EPD_ROT_90, EPD_ROT_180, EPD_ROT_270 } epd_rot_t;

// Refresh modes, ordered by strength: a stronger pending mode wins a merge.
// TRANSITION: a screen change without a flash; below GRAY, so the scheduler
// keeps its dirty list instead of widening it to the whole panel.
typedef enum { EPD_M_FAST, EPD_M_PARTIAL, EPD_M_TRANSITION, EPD_M_GRAY, EPD_M_FULL_CLEAN } epd_mode_t;

#ifdef __cplusplus
}
#endif
