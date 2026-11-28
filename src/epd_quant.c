// L8 -> packed panel planes. Walks the destination in panel-row order and
// assembles whole bytes; the L8 source is read with a constant index step per
// panel pixel, so rotation and mirroring cost nothing in the inner loop.
//
// 90 / 270 rotations (both recab panels): a panel row is a logical column.
// The word path (BW, BW_DITHER, GRAY4 with their standard tables, aligned
// L8) quantizes four panel rows per 32-bit L8 load and writes the plane
// through an internal-RAM tile; the block path (everything else, and the
// rect edges the word path leaves) stages the source block by block into
// internal RAM. Output identical to the row walk, which stays for 0 / 180
// and as the fallback (test_quant_block.c holds the old walk as reference).

#include "epd_quant.h"

#include <stdatomic.h>
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

// Row walk in tiles (any rotation)
static void quant_walk(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride, epd_rect_t p,
                       uint8_t *plane0, uint8_t *plane1, size_t pan_stride)
{
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

// ---- block path (90 / 270) ----
//
// With a 90 / 270 rotation (and any mirroring) the logical column depends on
// the panel row only and the logical row on the panel column only. A block of
// QB_ROWS panel rows x QB_COLS panel pixels therefore reads QB_COLS L8 rows,
// QB_ROWS consecutive bytes from each: one sequential copy per L8 row into
// internal RAM, instead of one PSRAM cache line per pixel read along a panel
// row (device B5: 44-60 ms per 3.97in frame, about 1 us per 8 pixels). The
// panel bytes are then built from the block, panel pixel k of the block at
// offset k * QB_ROWS, with every whole byte unrolled.
enum { QB_ROWS = 32, QB_COLS = 64 };
static uint8_t s_block[QB_ROWS * QB_COLS];   // .bss: internal RAM on the device
static atomic_flag s_block_busy = ATOMIC_FLAG_INIT;

#define QB(k) s[(k) * QB_ROWS]

static inline uint8_t byte_bw(const uint8_t *s, const uint8_t *lut)
{
    return (uint8_t)(lut[QB(0)] << 7 | lut[QB(1)] << 6 | lut[QB(2)] << 5 | lut[QB(3)] << 4 |
                     lut[QB(4)] << 3 | lut[QB(5)] << 2 | lut[QB(6)] << 1 | lut[QB(7)]);
}

// thr: the Bayer row of the panel row; the byte starts on a multiple of 8
static inline uint8_t byte_dither(const uint8_t *s, const uint8_t *lut, const uint8_t *thr)
{
    const uint8_t t0 = thr[0], t1 = thr[1], t2 = thr[2], t3 = thr[3];
    return (uint8_t)((lut[QB(0)] > t0) << 7 | (lut[QB(1)] > t1) << 6 | (lut[QB(2)] > t2) << 5 |
                     (lut[QB(3)] > t3) << 4 | (lut[QB(4)] > t0) << 3 | (lut[QB(5)] > t1) << 2 |
                     (lut[QB(6)] > t2) << 1 | (lut[QB(7)] > t3));
}

static inline void byte_gray2(const uint8_t *s, const uint8_t *lut, uint8_t *b0, uint8_t *b1)
{
    unsigned lo = 0, hi = 0;
    for (int k = 0; k < 8; k++) {
        const unsigned lv = lut[QB(k)];
        lo = lo << 1 | (lv & 1u);
        hi = hi << 1 | (lv >> 1);
    }
    *b0 = (uint8_t)lo;
    *b1 = (uint8_t)hi;
}

// One panel row of the block: pixels [x0, x1), s = the block bytes of pixel x0
static void block_row(const epd_quant_t *q, const uint8_t *s, int x0, int x1, int py, uint8_t *d0,
                      uint8_t *d1)
{
    const uint8_t *lut = q->lut;
    if (q->mode == EPD_Q_GRAY4) {
        int px = x0;
        if (px & 1) {
            d0[px >> 1] = (uint8_t)((d0[px >> 1] & 0xF0) | lut[s[0]]);
            px++;
            s += QB_ROWS;
        }
        for (; px + 1 < x1; px += 2, s += 2 * QB_ROWS) {
            d0[px >> 1] = (uint8_t)(lut[QB(0)] << 4 | lut[QB(1)]);
        }
        if (px < x1) {
            d0[px >> 1] = (uint8_t)((d0[px >> 1] & 0x0F) | (lut[s[0]] << 4));
        }
        return;
    }
    const uint8_t *thr = q->mode == EPD_Q_BW_DITHER ? s_bayer[py & 3] : s_no_dither;
    // partial bytes at the edges: the generic row code, reading the block
    const int a = (x0 + 7) & ~7, b = x1 & ~7;
    if (a >= b) {
        if (q->mode == EPD_Q_GRAY2) {
            row_gray2(d0, d1, x0, x1, s, 0, QB_ROWS, lut);
        } else {
            row_1bpp(d0, x0, x1, s, 0, QB_ROWS, lut, thr);
        }
        return;
    }
    if (q->mode == EPD_Q_GRAY2) {
        if (x0 < a) {
            row_gray2(d0, d1, x0, a, s, 0, QB_ROWS, lut);
        }
        const uint8_t *e = s + (a - x0) * QB_ROWS;
        for (int bx = a >> 3; bx < b >> 3; bx++, e += 8 * QB_ROWS) {
            byte_gray2(e, lut, &d0[bx], &d1[bx]);
        }
        if (b < x1) {
            row_gray2(d0, d1, b, x1, s, (ptrdiff_t)(b - x0) * QB_ROWS, QB_ROWS, lut);
        }
        return;
    }
    if (x0 < a) {
        row_1bpp(d0, x0, a, s, 0, QB_ROWS, lut, thr);
    }
    const uint8_t *e = s + (a - x0) * QB_ROWS;
    if (q->mode == EPD_Q_BW_DITHER) {
        for (int bx = a >> 3; bx < b >> 3; bx++, e += 8 * QB_ROWS) {
            d0[bx] = byte_dither(e, lut, thr);
        }
    } else {
        for (int bx = a >> 3; bx < b >> 3; bx++, e += 8 * QB_ROWS) {
            d0[bx] = byte_bw(e, lut);
        }
    }
    if (b < x1) {
        row_1bpp(d0, b, x1, s, (ptrdiff_t)(b - x0) * QB_ROWS, QB_ROWS, lut, thr);
    }
}

static void quant_block(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride, epd_rect_t p,
                        uint8_t *plane0, uint8_t *plane1, size_t pan_stride, uint8_t *blk)
{
    // lx = lx0 + dlx * (py - p.y), ly = ly0 + dly * (px - p.x); dlx, dly = +-1
    int lx0, ly0, lxr, lyr, lxc, lyc;
    to_logical(q, p.x, p.y, &lx0, &ly0);
    to_logical(q, p.x, p.y + 1, &lxr, &lyr);
    to_logical(q, p.x + 1, p.y, &lxc, &lyc);
    const int dlx = lxr - lx0, dly = lyc - ly0;
    const int x0 = p.x, x1 = p.x + p.w, y1 = p.y + p.h;
    for (int by = p.y; by < y1; by += QB_ROWS) {
        const int n = y1 - by < QB_ROWS ? y1 - by : QB_ROWS;
        const int la = lx0 + dlx * (by - p.y);
        const int lmin = dlx > 0 ? la : la - (n - 1);   // the n logical columns of these rows
        for (int cx = x0; cx < x1;) {
            int cx1 = (cx / QB_COLS + 1) * QB_COLS;
            cx1 = cx1 < x1 ? cx1 : x1;
            const uint8_t *src = l8 + (ptrdiff_t)(ly0 + dly * (cx - x0)) * (ptrdiff_t)l8_stride + lmin;
            const ptrdiff_t src_step = (ptrdiff_t)dly * (ptrdiff_t)l8_stride;
            for (int k = 0; k < cx1 - cx; k++, src += src_step) {
                memcpy(blk + k * QB_ROWS, src, (size_t)n);
            }
            for (int r = 0; r < n; r++) {
                const int py = by + r;
                const size_t off = (size_t)py * pan_stride;
                block_row(q, blk + (dlx > 0 ? r : n - 1 - r), cx, cx1, py, plane0 + off,
                          plane1 ? plane1 + off : NULL);
            }
            cx = cx1;
        }
    }
}

// ---- word path (90 / 270, the recab formats) ----
//
// With the standard tables of epd_quant_set_mode the level of a pixel is a
// bit test (BW: bit 7), a compare (BW_DITHER: v > Bayer threshold) or the
// high nibble (GRAY4), on v or on ~v when inverted. Four consecutive L8 bytes
// of one row are four consecutive panel rows at one panel pixel, so a 32-bit
// load quantizes four pixels at once:
//   BW / BW_DITHER: bit 7 of each byte lane is the pixel's bit; shifted right
//   by the pixel's position in its panel byte (k = 0..7, MSB first) it lands
//   on its own bit inside the same lane, so ORing the eight L8 rows of one
//   panel byte column leaves, in byte lane i, the finished panel byte of the
//   lane's panel row.
//   GRAY4: (a & 0xF0) | (b >> 4) per lane, a and b the L8 rows of the two
//   pixels of a panel byte.
// Little-endian lanes: byte lane i of a word is the byte at address + i.
// The L8 rows are read in order (eight sequential streams), the panel planes
// are written one byte per panel row. Rect edges that do not fill a word or a
// panel byte go through the block path.

#define WL 0x80808080u

typedef enum { QW_NONE, QW_BW, QW_DITHER, QW_GRAY4 } qword_t;

// The word path covers this table and mode, inverted or not (*inv)
static qword_t word_kind(const epd_quant_t *q, bool *inv)
{
    qword_t k;
    switch (q->mode) {
    case EPD_Q_BW:        k = QW_BW; break;
    case EPD_Q_BW_DITHER: k = QW_DITHER; break;
    case EPD_Q_GRAY4:     k = QW_GRAY4; break;
    default:              return QW_NONE;
    }
    *inv = q->lut[0] != level_of(q->mode, 0);
    for (int v = 0; v < 256; v++) {
        if (q->lut[v] != level_of(q->mode, (uint8_t)(*inv ? 255 - v : v))) {
            return QW_NONE;
        }
    }
    return k;
}

// Per byte lane: 0x80 where x > t (unsigned), t1 = t + 1 per lane (x >= t1)
static inline uint32_t lanes_gt(uint32_t x, uint32_t t1)
{
    const uint32_t ge7 = ((x | WL) - (t1 & ~WL)) & WL;     // low 7 bits: x >= t1, no borrow between lanes
    return ((x & ~t1) | (~(x ^ t1) & ge7)) & WL;
}

static inline uint32_t load32(const uint8_t *p)
{
    return *(const uint32_t *)(const void *)p;     // 4-byte aligned: checked by the caller
}

// Output tile in internal RAM (s_block): QW_ROWS panel rows x QW_BYTES plane bytes,
// copied out one whole run of bytes per panel row. Writing the plane one
// byte per panel row straight into PSRAM evicted its cache lines between two
// columns, so every line was fetched again for each byte (device: 30 ms per
// 3.97in frame instead of the 9 ms the reads alone take).
enum { QW_ROWS = 64, QW_BYTES = 32 };
_Static_assert(QW_ROWS * QW_BYTES <= QB_ROWS * QB_COLS, "the output tile shares s_block");

// 8 L8 rows (one panel byte column of a 1 bpp plane) x 4 panel rows. Paper
// and ink of a UI frame are mostly whole words of 0xFF or 0x00: those are
// all white or all black in both modes (the Bayer thresholds are 8..248), so
// only words with an edge or a gray take the per-lane compare.
static inline uint32_t word_1bpp(qword_t kind, const uint8_t *r, ptrdiff_t ss, uint32_t xv, const uint32_t *t1)
{
    const uint32_t w0 = load32(r) ^ xv, w1 = load32(r + ss) ^ xv, w2 = load32(r + 2 * ss) ^ xv;
    const uint32_t w3 = load32(r + 3 * ss) ^ xv, w4 = load32(r + 4 * ss) ^ xv, w5 = load32(r + 5 * ss) ^ xv;
    const uint32_t w6 = load32(r + 6 * ss) ^ xv, w7 = load32(r + 7 * ss) ^ xv;
    const uint32_t all = w0 & w1 & w2 & w3 & w4 & w5 & w6 & w7;
    const uint32_t any = w0 | w1 | w2 | w3 | w4 | w5 | w6 | w7;
    if (all == 0xFFFFFFFFu) {
        return 0xFFFFFFFFu;
    }
    if (any == 0) {
        return 0;
    }
    if (kind == QW_DITHER) {
        return lanes_gt(w0, t1[0]) | lanes_gt(w1, t1[1]) >> 1 | lanes_gt(w2, t1[2]) >> 2 |
               lanes_gt(w3, t1[3]) >> 3 | lanes_gt(w4, t1[0]) >> 4 | lanes_gt(w5, t1[1]) >> 5 |
               lanes_gt(w6, t1[2]) >> 6 | lanes_gt(w7, t1[3]) >> 7;
    }
    return (w0 & WL) | (w1 & WL) >> 1 | (w2 & WL) >> 2 | (w3 & WL) >> 3 | (w4 & WL) >> 4 | (w5 & WL) >> 5 |
           (w6 & WL) >> 6 | (w7 & WL) >> 7;
}

// The core rect: c.h a multiple of 4 whose logical columns start on a word,
// c.x and c.w whole panel bytes
static void quant_word(const epd_quant_t *q, qword_t kind, bool inv, const uint8_t *l8, size_t l8_stride,
                       epd_rect_t c, uint8_t *plane0, size_t pan_stride, uint8_t *tile)
{
    int lx0, ly0, lxr, lyr, lxc, lyc;
    to_logical(q, c.x, c.y, &lx0, &ly0);
    to_logical(q, c.x, c.y + 1, &lxr, &lyr);
    to_logical(q, c.x + 1, c.y, &lxc, &lyc);
    const int dlx = lxr - lx0, dly = lyc - ly0;
    const ptrdiff_t ss = (ptrdiff_t)dly * (ptrdiff_t)l8_stride;    // L8 step to the next panel pixel
    const uint32_t xv = inv ? 0xFFFFFFFFu : 0u;
    const int ppb = kind == QW_GRAY4 ? 2 : 8;                       // panel pixels per plane byte
    // Bayer threshold + 1 per lane, for panel pixel column k & 3 (c.x is a
    // multiple of 8). Lane i of every word sits on a panel row of the same
    // phase: words start on multiples of 4 logical columns.
    uint32_t t1[4] = {0, 0, 0, 0};
    if (kind == QW_DITHER) {
        const int lw = dlx > 0 ? lx0 : lx0 - (c.h - 1);    // a multiple of 4
        for (int k = 0; k < 4; k++) {
            for (int i = 0; i < 4; i++) {
                const int py = c.y + (lw + i - lx0) * dlx;
                t1[k] |= (uint32_t)(s_bayer[py & 3][k] + 1u) << (8 * i);
            }
        }
    }
    const int b0 = c.x / ppb, b1 = (c.x + c.w) / ppb;     // plane bytes [b0, b1)
    for (int y = c.y; y < c.y + c.h; y += QW_ROWS) {
        const int th = c.y + c.h - y < QW_ROWS ? c.y + c.h - y : QW_ROWS;     // a multiple of 4
        const int lxy = lx0 + (y - c.y) * dlx;              // logical column of panel row y
        const int lmin = dlx > 0 ? lxy : lxy - (th - 1);    // a multiple of 4
        // lane i of word g is tile row r0 + i * dlx
        const int r0 = dlx > 0 ? 0 : th - 1;
        const ptrdiff_t lane = dlx > 0 ? QW_BYTES : -QW_BYTES;
        for (int bb = b0; bb < b1; bb += QW_BYTES) {
            const int nb = b1 - bb < QW_BYTES ? b1 - bb : QW_BYTES;
            for (int j = 0; j < nb; j++) {
                const int px = (bb + j) * ppb;
                const uint8_t *r = l8 + (ptrdiff_t)(ly0 + dly * (px - c.x)) * (ptrdiff_t)l8_stride + lmin;
                uint8_t *o = tile + r0 * QW_BYTES + j;
                for (int g = 0; g < th / 4; g++, r += 4, o += 4 * lane) {
                    uint32_t w;
                    if (kind == QW_GRAY4) {
                        w = ((load32(r) ^ xv) & 0xF0F0F0F0u) | (((load32(r + ss) ^ xv) >> 4) & 0x0F0F0F0Fu);
                    } else {
                        w = word_1bpp(kind, r, ss, xv, t1);
                    }
                    o[0] = (uint8_t)w;
                    o[lane] = (uint8_t)(w >> 8);
                    o[2 * lane] = (uint8_t)(w >> 16);
                    o[3 * lane] = (uint8_t)(w >> 24);
                }
            }
            for (int t = 0; t < th; t++) {
                memcpy(plane0 + (size_t)(y + t) * pan_stride + bb, tile + t * QW_BYTES, (size_t)nb);
            }
        }
    }
}

// Splits p into the word core and the edges around it (block path). False
// when there is no core (the caller then takes the block path for all of p).
static bool quant_rot_words(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride, epd_rect_t p,
                            uint8_t *plane0, size_t pan_stride, uint8_t *blk)
{
    bool inv = false;
    const qword_t kind = word_kind(q, &inv);
    if (kind == QW_NONE || ((uintptr_t)l8 & 3u) || (l8_stride & 3u)) {
        return false;
    }
    // panel x: whole panel bytes
    const int unit = kind == QW_GRAY4 ? 2 : 8;
    const int ax = (p.x + unit - 1) / unit * unit, bx = (p.x + p.w) / unit * unit;
    // panel y: rows whose logical columns fill whole words
    int lx0, ly0, lxr, lyr;
    to_logical(q, p.x, p.y, &lx0, &ly0);
    to_logical(q, p.x, p.y + 1, &lxr, &lyr);
    const int dlx = lxr - lx0;
    const int lmin = dlx > 0 ? lx0 : lx0 - (p.h - 1), lmax = lmin + p.h - 1;
    const int la = (lmin + 3) & ~3, lb = (lmax + 1) & ~3;     // core logical columns [la, lb)
    if (ax >= bx || la >= lb) {
        return false;
    }
    const int cy0 = dlx > 0 ? p.y + (la - lx0) : p.y + (lx0 - (lb - 1));
    const int cy1 = cy0 + (lb - la);
    const epd_rect_t core = {(int16_t)ax, (int16_t)cy0, (int16_t)(bx - ax), (int16_t)(cy1 - cy0)};
    quant_word(q, kind, inv, l8, l8_stride, core, plane0, pan_stride, blk);    // then the edges reuse blk
    const epd_rect_t edge[4] = {
        {p.x, p.y, p.w, (int16_t)(cy0 - p.y)},                                  // above the core
        {p.x, (int16_t)cy1, p.w, (int16_t)(p.y + p.h - cy1)},                   // below
        {p.x, (int16_t)cy0, (int16_t)(ax - p.x), (int16_t)(cy1 - cy0)},         // left
        {(int16_t)bx, (int16_t)cy0, (int16_t)(p.x + p.w - bx), (int16_t)(cy1 - cy0)},   // right
    };
    for (int i = 0; i < 4; i++) {
        if (edge[i].w > 0 && edge[i].h > 0) {
            quant_block(q, l8, l8_stride, edge[i], plane0, NULL, pan_stride, blk);
        }
    }
    return true;
}

void epd_quant_rect(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride,
                    epd_rect_t logical, uint8_t *plane0, uint8_t *plane1,
                    size_t pan_stride)
{
    epd_rect_t p = epd_quant_map_rect(q, logical);
    if (p.w <= 0 || p.h <= 0) return;
    if (q->mode == EPD_Q_GRAY2 && plane1 == NULL) return;
    // One block buffer: a second caller at the same time (another task)
    // takes the row walk rather than waiting
    if ((q->rot == EPD_ROT_90 || q->rot == EPD_ROT_270) && !atomic_flag_test_and_set(&s_block_busy)) {
        if (!quant_rot_words(q, l8, l8_stride, p, plane0, pan_stride, s_block)) {
            quant_block(q, l8, l8_stride, p, plane0, plane1, pan_stride, s_block);
        }
        atomic_flag_clear(&s_block_busy);
        return;
    }
    quant_walk(q, l8, l8_stride, p, plane0, plane1, pan_stride);
}

