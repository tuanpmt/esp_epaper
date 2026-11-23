#pragma once

// Plane compare: did anything change inside the windows of a run? Pure C, no
// I/O: host-testable. Planes are packed rows, MSB first (1 bpp: bit 7 is the
// leftmost pixel; 4 bpp: the high nibble is the leftmost pixel). Rects are in
// panel pixels; bpp is 1 or 4.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Does any pixel differ between a and b inside the rects? Rects are clipped to
// the plane rows' left edge only (x < 0 starts at 0); the caller keeps them on
// the plane. n == 0 compares nothing (false).
bool epd_plane_differs(const uint8_t *a, const uint8_t *b, size_t stride, uint8_t bpp,
                       const epd_rect_t *rects, uint8_t n);

// Copy of the plane last sent to the controller RAM (3.97in: RAM 0x24). A run
// whose windows all equal the copy changes nothing on the panel and can be
// skipped. The copy is taken just before the transfer and the transfer is
// sent from the copy, so it is exactly what the controller holds, even when
// the renderer changes the live plane during the refresh.
#define EPD_PLANE_MAX_RECTS 8
typedef struct {
    uint8_t *buf;           // caller-owned, stride x h bytes
    size_t stride;
    uint16_t w, h;
    uint8_t bpp;
    bool valid;             // buf equals the controller RAM
} epd_plane_sent_t;

// Starts invalid: the controller RAM is unknown until the first transfer.
void epd_plane_sent_init(epd_plane_sent_t *s, uint8_t *buf, uint16_t w, uint16_t h, size_t stride, uint8_t bpp);
// The controller RAM is unknown again (reset, failed transfer, other image written).
void epd_plane_sent_invalidate(epd_plane_sent_t *s);
// Which windows of a run differ from the copy: bit i set = rects[i] differs.
// n == 0 means the whole plane (bit 0). While the copy is invalid every
// window differs. At most EPD_PLANE_MAX_RECTS rects are looked at.
uint8_t epd_plane_sent_diff(const epd_plane_sent_t *s, const uint8_t *plane, const epd_rect_t *rects, uint8_t n);
// The plane is about to be sent: copy it and mark the copy valid. Returns the
// number of pixels that differ from the previous copy (every pixel when it
// was invalid). Send s->buf, not the live plane.
uint32_t epd_plane_sent_take(epd_plane_sent_t *s, const uint8_t *plane);

#ifdef __cplusplus
}
#endif
