#pragma once

// Dirty rectangle set in panel coordinates. Panel-independent: size and the
// x alignment (8 for 1 bpp, 2 for 4 bpp) are parameters. No ESP-IDF
// dependency: host-testable.

#include <stdbool.h>
#include <stdint.h>

#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_DIRTY_MAX 8

typedef struct {
    epd_rect_t r[EPD_DIRTY_MAX];
    uint8_t n;
    uint16_t pan_w, pan_h;
    uint8_t align;              // align on the panel byte axis (x), 8 for 1 bpp
    uint32_t full_area_pct;     // union area above this % of the screen -> one full rect
} epd_dirty_t;

void epd_dirty_init(epd_dirty_t *d, uint16_t pan_w, uint16_t pan_h, uint8_t align, uint32_t full_area_pct);
// Clip to the panel, align x outwards, merge with overlapping/edge-adjacent rects.
// Empty or fully off-panel rects are ignored. No-op once the set is full.
void epd_dirty_add(epd_dirty_t *d, epd_rect_t panel_rect);
void epd_dirty_add_all(epd_dirty_t *d);
bool epd_dirty_is_full(const epd_dirty_t *d);
// Moves the set out (merging down to at most max rects) and clears it.
// Returns the number of rects written; max == 0 returns 0 and keeps the set.
uint8_t epd_dirty_take(epd_dirty_t *d, epd_rect_t *out, uint8_t max);

#ifdef __cplusplus
}
#endif
