#pragma once

// L8 (LVGL 8-bit luminance) to packed panel planes: rotation, mirroring,
// threshold / ordered dithering / gray levels, invert. Panel-independent:
// sizes, orientation and bit depth are parameters. No ESP-IDF dependency.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { EPD_Q_BW, EPD_Q_BW_DITHER, EPD_Q_GRAY2, EPD_Q_GRAY4 } epd_qmode_t;

typedef struct {
    uint16_t log_w, log_h;          // LVGL buffer size (e.g. portrait 480x800)
    uint16_t pan_w, pan_h;          // native panel (e.g. landscape 800x480)
    epd_rot_t rot;                  // logical -> panel
    bool mirror_x, mirror_y;        // applied after rotation
    // L8 -> level (0..(1<<bpp)-1), invert folded in. In EPD_Q_BW_DITHER it
    // holds the (possibly inverted) L8 intensity that is compared against
    // the Bayer threshold instead.
    uint8_t lut[256];
    epd_qmode_t mode;
} epd_quant_t;

// Mode starts as EPD_Q_BW, not inverted, no mirroring.
void epd_quant_init(epd_quant_t *q, uint16_t log_w, uint16_t log_h,
                    uint16_t pan_w, uint16_t pan_h, epd_rot_t rot);

// Selects the quantization mode and rebuilds the lut.
void epd_quant_set_mode(epd_quant_t *q, epd_qmode_t mode, bool invert);

// Logical rect -> panel rect. The input is clipped to the logical buffer
// first; an empty result has w = h = 0.
epd_rect_t epd_quant_map_rect(const epd_quant_t *q, epd_rect_t logical);

// Convert one logical rect of the L8 buffer into packed panel planes. Only
// the panel pixels covered by the (clipped) rect are written.
// BW / BW_DITHER: 1 bpp, MSB first, bit 1 = white (SSD16xx), plane0 only.
// GRAY2: plane0 = LSB, plane1 = MSB (stock 4-gray overlay layout).
// GRAY4: 4 bpp in plane0, high nibble = left pixel.
// plane1 is only used (and may be NULL otherwise) in GRAY2.
void epd_quant_rect(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride,
                    epd_rect_t logical, uint8_t *plane0, uint8_t *plane1,
                    size_t pan_stride);

#ifdef __cplusplus
}
#endif
