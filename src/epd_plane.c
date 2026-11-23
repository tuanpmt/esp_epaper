// Plane compare: see epd_plane.h. Pure C, no I/O.

#include "epd_plane.h"

#include <string.h>

// One row of a rect, as a bit range [b0, b1) of the packed row (MSB first)
static bool row_differs(const uint8_t *a, const uint8_t *b, uint32_t b0, uint32_t b1)
{
    const uint32_t first = b0 / 8, last = (b1 - 1) / 8;
    uint8_t head = (uint8_t)(0xFFu >> (b0 % 8));
    const uint8_t tail = (uint8_t)(0xFFu << (7 - (b1 - 1) % 8));
    if (first == last) {
        return ((a[first] ^ b[first]) & head & tail) != 0;
    }
    if ((a[first] ^ b[first]) & head) {
        return true;
    }
    if ((a[last] ^ b[last]) & tail) {
        return true;
    }
    return last > first + 1 && memcmp(a + first + 1, b + first + 1, last - first - 1) != 0;
}

static bool rect_differs(const uint8_t *a, const uint8_t *b, size_t stride, uint8_t bpp, epd_rect_t r)
{
    int32_t x0 = r.x, y0 = r.y;
    const int32_t x1 = (int32_t)r.x + r.w, y1 = (int32_t)r.y + r.h;
    if (x0 < 0) {
        x0 = 0;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (x0 >= x1 || y0 >= y1) {
        return false;
    }
    const uint32_t b0 = (uint32_t)x0 * bpp, b1 = (uint32_t)x1 * bpp;
    for (int32_t y = y0; y < y1; y++) {
        const size_t at = (size_t)y * stride;
        if (row_differs(a + at, b + at, b0, b1)) {
            return true;
        }
    }
    return false;
}

bool epd_plane_differs(const uint8_t *a, const uint8_t *b, size_t stride, uint8_t bpp, const epd_rect_t *rects,
                       uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        if (rect_differs(a, b, stride, bpp, rects[i])) {
            return true;
        }
    }
    return false;
}

void epd_plane_sent_init(epd_plane_sent_t *s, uint8_t *buf, uint16_t w, uint16_t h, size_t stride, uint8_t bpp)
{
    memset(s, 0, sizeof(*s));
    s->buf = buf;
    s->w = w;
    s->h = h;
    s->stride = stride;
    s->bpp = bpp;
}

void epd_plane_sent_invalidate(epd_plane_sent_t *s)
{
    s->valid = false;
}

uint8_t epd_plane_sent_diff(const epd_plane_sent_t *s, const uint8_t *plane, const epd_rect_t *rects, uint8_t n)
{
    const epd_rect_t whole = {0, 0, (int16_t)s->w, (int16_t)s->h};
    if (n == 0) {
        rects = &whole;
        n = 1;
    }
    if (n > EPD_PLANE_MAX_RECTS) {
        n = EPD_PLANE_MAX_RECTS;
    }
    uint8_t mask = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (!s->valid || rect_differs(plane, s->buf, s->stride, s->bpp, rects[i])) {
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

// Pixels that differ in one byte (8 pixels at 1 bpp, 2 at 4 bpp)
static uint32_t byte_diff_px(uint8_t x, uint8_t bpp)
{
    if (bpp == 4) {
        return (uint32_t)((x & 0xF0) != 0) + (uint32_t)((x & 0x0F) != 0);
    }
    return (uint32_t)__builtin_popcount(x);
}

uint32_t epd_plane_sent_take(epd_plane_sent_t *s, const uint8_t *plane)
{
    const size_t size = s->stride * s->h;
    uint32_t changed = 0;
    if (!s->valid) {
        changed = (uint32_t)s->w * s->h;
        memcpy(s->buf, plane, size);
    } else {
        // compare and copy in one pass; each byte read once, as the renderer
        // may write the live plane meanwhile
        for (size_t i = 0; i < size; i++) {
            const uint8_t p = plane[i];
            const uint8_t x = (uint8_t)(s->buf[i] ^ p);
            if (x) {
                changed += byte_diff_px(x, s->bpp);
                s->buf[i] = p;
            }
        }
    }
    s->valid = true;
    return changed;
}
