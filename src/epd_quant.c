// L8 -> packed panel planes. Walks the destination in panel-row order and
// assembles whole bytes; the L8 source is read with a constant index step per
// panel pixel, so rotation and mirroring cost nothing in the inner loop.

#include "epd_quant.h"

#include <string.h>

// Bayer 4x4 thresholds b*16 + 8: a value v turns white where v > threshold.
// Mid gray 128 lights exactly half the cells; 0 and 255 never dither
// (0 > 8 and 255 > 248 are fixed), so hinted 1 bpp text stays crisp.
static const uint8_t s_bayer[4][4] = {
    {  8, 136,  40, 168 },
    { 200,  72, 232, 104 },
    {  56, 184,  24, 152 },
    { 248, 120, 216,  88 },
};

static const uint8_t s_no_dither[4] = { 0, 0, 0, 0 };

static uint8_t level_of(epd_qmode_t mode, uint8_t v)
{
    switch (mode) {
    case EPD_Q_BW:        return v >= 128;
    case EPD_Q_BW_DITHER: return v;   // intensity, compared against Bayer
    case EPD_Q_GRAY2:     return (uint8_t)((v >= 43) + (v >= 128) + (v >= 213));
    case EPD_Q_GRAY4:     return v >> 4;
    }
    return 0;
}

void epd_quant_set_mode(epd_quant_t *q, epd_qmode_t mode, bool invert)
{
    q->mode = mode;
    for (int v = 0; v < 256; v++) {
        q->lut[v] = level_of(mode, (uint8_t)(invert ? 255 - v : v));
    }
}

void epd_quant_init(epd_quant_t *q, uint16_t log_w, uint16_t log_h,
                    uint16_t pan_w, uint16_t pan_h, epd_rot_t rot)
{
    memset(q, 0, sizeof(*q));
    q->log_w = log_w;
    q->log_h = log_h;
    q->pan_w = pan_w;
    q->pan_h = pan_h;
    q->rot = rot;
    epd_quant_set_mode(q, EPD_Q_BW, false);
}

// Clip to the logical buffer; false when nothing is left
static bool clip(const epd_quant_t *q, epd_rect_t r, int *x0, int *y0, int *x1, int *y1)
{
    if (r.w <= 0 || r.h <= 0) return false;
    *x0 = r.x < 0 ? 0 : r.x;
    *y0 = r.y < 0 ? 0 : r.y;
    *x1 = (int)r.x + r.w > q->log_w ? q->log_w : (int)r.x + r.w;
    *y1 = (int)r.y + r.h > q->log_h ? q->log_h : (int)r.y + r.h;
    return *x0 < *x1 && *y0 < *y1;
}

static void to_panel(const epd_quant_t *q, int lx, int ly, int *px, int *py)
{
    int x, y;
    switch (q->rot) {
    case EPD_ROT_90:  x = ly;                y = q->pan_h - 1 - lx; break;
    case EPD_ROT_180: x = q->pan_w - 1 - lx; y = q->pan_h - 1 - ly; break;
    case EPD_ROT_270: x = q->pan_w - 1 - ly; y = lx;                break;
    default:          x = lx;                y = ly;                break;
    }
    *px = q->mirror_x ? q->pan_w - 1 - x : x;
    *py = q->mirror_y ? q->pan_h - 1 - y : y;
}

static void to_logical(const epd_quant_t *q, int px, int py, int *lx, int *ly)
{
    int x = q->mirror_x ? q->pan_w - 1 - px : px;
    int y = q->mirror_y ? q->pan_h - 1 - py : py;
    switch (q->rot) {
    case EPD_ROT_90:  *lx = q->pan_h - 1 - y; *ly = x;                break;
    case EPD_ROT_180: *lx = q->pan_w - 1 - x; *ly = q->pan_h - 1 - y; break;
    case EPD_ROT_270: *lx = y;                *ly = q->pan_w - 1 - x; break;
    default:          *lx = x;                *ly = y;                break;
    }
}

epd_rect_t epd_quant_map_rect(const epd_quant_t *q, epd_rect_t logical)
{
    int x0, y0, x1, y1;
    if (!clip(q, logical, &x0, &y0, &x1, &y1)) return (epd_rect_t){0, 0, 0, 0};
    int ax, ay, bx, by;
    to_panel(q, x0, y0, &ax, &ay);
    to_panel(q, x1 - 1, y1 - 1, &bx, &by);
    int px0 = ax < bx ? ax : bx, px1 = ax < bx ? bx : ax;
    int py0 = ay < by ? ay : by, py1 = ay < by ? by : ay;
    return (epd_rect_t){(int16_t)px0, (int16_t)py0, (int16_t)(px1 - px0 + 1), (int16_t)(py1 - py0 + 1)};
}

// 1 bpp row. BW: lut holds 0/1 and thr is all zero; BW_DITHER: lut holds
// the intensity and thr is the Bayer row. Partial edge bytes keep their
// untouched bits.
static void row_1bpp(uint8_t *dst, int x0, int x1, const uint8_t *l8, ptrdiff_t idx,
                     ptrdiff_t step, const uint8_t *lut, const uint8_t *thr)
{
    int px = x0;
    while (px < x1) {
        int bx = px >> 3;
        int end = (bx + 1) * 8 < x1 ? (bx + 1) * 8 : x1;
        uint8_t bits = 0, mask = 0;
        for (; px < end; px++, idx += step) {
            uint8_t m = (uint8_t)(0x80u >> (px & 7));
            mask |= m;
            if (lut[l8[idx]] > thr[px & 3]) bits |= m;
        }
        dst[bx] = mask == 0xFF ? bits : (uint8_t)((dst[bx] & ~mask) | bits);
    }
}

// 2 levels per pixel split over two 1 bpp planes: d0 = LSB, d1 = MSB
static void row_gray2(uint8_t *d0, uint8_t *d1, int x0, int x1, const uint8_t *l8,
                      ptrdiff_t idx, ptrdiff_t step, const uint8_t *lut)
{
    int px = x0;
    while (px < x1) {
        int bx = px >> 3;
        int end = (bx + 1) * 8 < x1 ? (bx + 1) * 8 : x1;
        uint8_t b0 = 0, b1 = 0, mask = 0;
        for (; px < end; px++, idx += step) {
            uint8_t m = (uint8_t)(0x80u >> (px & 7));
            uint8_t lv = lut[l8[idx]];
            mask |= m;
            if (lv & 1) b0 |= m;
            if (lv & 2) b1 |= m;
        }
        if (mask == 0xFF) {
            d0[bx] = b0;
            d1[bx] = b1;
        } else {
            d0[bx] = (uint8_t)((d0[bx] & ~mask) | b0);
            d1[bx] = (uint8_t)((d1[bx] & ~mask) | b1);
        }
    }
}

// 4 bpp packed, high nibble = left (even) pixel
static void row_gray4(uint8_t *dst, int x0, int x1, const uint8_t *l8, ptrdiff_t idx,
                      ptrdiff_t step, const uint8_t *lut)
{
    int px = x0;
    if (px & 1) {
        dst[px >> 1] = (uint8_t)((dst[px >> 1] & 0xF0) | lut[l8[idx]]);
        px++;
        idx += step;
    }
    for (; px + 1 < x1; px += 2, idx += 2 * step) {
        dst[px >> 1] = (uint8_t)((lut[l8[idx]] << 4) | lut[l8[idx + step]]);
    }
    if (px < x1) {
        dst[px >> 1] = (uint8_t)((dst[px >> 1] & 0x0F) | (lut[l8[idx]] << 4));
    }
}

void epd_quant_rect(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride,
                    epd_rect_t logical, uint8_t *plane0, uint8_t *plane1,
                    size_t pan_stride)
{
    epd_rect_t p = epd_quant_map_rect(q, logical);
    if (p.w <= 0 || p.h <= 0) return;
    if (q->mode == EPD_Q_GRAY2 && plane1 == NULL) return;

    // Source index of the first panel pixel, and the index deltas for one
    // step along a panel row and down one panel row (constant per mapping).
    int lx, ly, cx, cy, rx, ry;
    to_logical(q, p.x, p.y, &lx, &ly);
    to_logical(q, p.x + 1, p.y, &cx, &cy);
    to_logical(q, p.x, p.y + 1, &rx, &ry);
    const ptrdiff_t stride = (ptrdiff_t)l8_stride;
    const ptrdiff_t col_step = (ptrdiff_t)(cy - ly) * stride + (cx - lx);
    const ptrdiff_t row_step = (ptrdiff_t)(ry - ly) * stride + (rx - lx);
    ptrdiff_t idx = (ptrdiff_t)ly * stride + lx;

    // Tiled walk. With a 90/270 rotation a panel row reads one logical
    // column: every pixel in another L8 row (cache line). Row by row, the
    // next panel row needs the same lines again, more than the data cache
    // holds, so almost every read missed (device: 270 ms per full frame,
    // competing with the waveform for PSRAM). In tiles of TILE_ROWS panel
    // rows x TILE_COLS panel pixels the source lines are reused while
    // cached: 64 panel rows use 64 consecutive bytes (one cache line) of each
    // L8 row, 128 columns keep about 8 KB of source plus 4 KB of output hot.
    // Tile edges are multiples of 8 pixels, so only the rect's own edges
    // take the partial-byte paths. Output identical to a row walk.
    enum { TILE_ROWS = 64, TILE_COLS = 128 };
    const int x0 = p.x, x1 = p.x + p.w, y1 = p.y + p.h;
    for (int by = p.y; by < y1; by += TILE_ROWS) {
        const int by1 = by + TILE_ROWS < y1 ? by + TILE_ROWS : y1;
        for (int cx = x0; cx < x1;) {
            int cx1 = (cx / TILE_COLS + 1) * TILE_COLS;
            cx1 = cx1 < x1 ? cx1 : x1;
            for (int py = by; py < by1; py++) {
                const size_t off = (size_t)py * pan_stride;
                const ptrdiff_t i = idx + (ptrdiff_t)(py - p.y) * row_step + (ptrdiff_t)(cx - x0) * col_step;
                switch (q->mode) {
                case EPD_Q_BW:
                    row_1bpp(plane0 + off, cx, cx1, l8, i, col_step, q->lut, s_no_dither);
                    break;
                case EPD_Q_BW_DITHER:
                    row_1bpp(plane0 + off, cx, cx1, l8, i, col_step, q->lut, s_bayer[py & 3]);
                    break;
                case EPD_Q_GRAY2:
                    row_gray2(plane0 + off, plane1 + off, cx, cx1, l8, i, col_step, q->lut);
                    break;
                case EPD_Q_GRAY4:
                    row_gray4(plane0 + off, cx, cx1, l8, i, col_step, q->lut);
                    break;
                }
            }
            cx = cx1;
        }
    }
}
