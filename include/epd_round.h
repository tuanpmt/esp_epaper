#pragma once

// Rounding of LVGL invalidation areas (pure C, host-tested): pad for glyph
// edges, clip to the screen, and align the axis that becomes the panel's byte
// axis after rotation, so partial windows never cut a byte.

#include <stdint.h>
#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// a: logical rect on a hor x ver screen. Returns an empty rect (w or h <= 0)
// when nothing of it is on screen.
epd_rect_t epd_round_rect(epd_rect_t a, uint16_t hor, uint16_t ver, epd_rot_t rot, uint8_t align, uint8_t pad_x,
                          uint8_t pad_y);

#ifdef __cplusplus
}
#endif
