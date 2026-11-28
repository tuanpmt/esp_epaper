#include "epd_round.h"

static int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

epd_rect_t epd_round_rect(epd_rect_t a, uint16_t hor, uint16_t ver, epd_rot_t rot, uint8_t align, uint8_t pad_x,
                          uint8_t pad_y)
{
    const epd_rect_t empty = {0, 0, 0, 0};
    if (a.w <= 0 || a.h <= 0) {
        return empty;
    }
    int32_t x0 = (int32_t)a.x - pad_x, x1 = (int32_t)a.x + a.w + pad_x;   // [x0, x1)
    int32_t y0 = (int32_t)a.y - pad_y, y1 = (int32_t)a.y + a.h + pad_y;
    if (align > 1) {
        // ROT_0/180: panel x = logical x; ROT_90/270: panel x = logical y.
        // Screen sizes are multiples of align on that axis (800 for the SSD1677).
        if (rot == EPD_ROT_0 || rot == EPD_ROT_180) {
            x0 -= ((x0 % align) + align) % align;
            x1 += (align - ((x1 % align) + align) % align) % align;
        } else {
            y0 -= ((y0 % align) + align) % align;
            y1 += (align - ((y1 % align) + align) % align) % align;
        }
    }
    x0 = clamp(x0, 0, hor);
    x1 = clamp(x1, 0, hor);
    y0 = clamp(y0, 0, ver);
    y1 = clamp(y1, 0, ver);
    if (x1 <= x0 || y1 <= y0) {
        return empty;
    }
    return (epd_rect_t){(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0), (int16_t)(y1 - y0)};
}
