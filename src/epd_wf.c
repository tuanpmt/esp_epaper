// Waveform engine for 4 bpp planes (see epd_wf.h). Own code; the table
// format and the bus bit order follow the vendor firmware of the Metalio
// E-Ink 4 Plus.

#include <string.h>
#include "epd_wf.h"

int epd_wf_action(const epd_wf_phases_t *w, int phase, int from, int to)
{
    const uint8_t b = w->lut[((size_t)phase * EPD_WF_LEVELS + (size_t)to) * 4 + (size_t)(from / 4)];
    return (b >> (6 - 2 * (from % 4))) & 3;
}

// Built 32 bits at a time: for a fixed cur byte c and next high nibble h the
// 16 entries n = (h << 4) | l differ only in the right pixel's bits.
void epd_wf_pair_lut(const epd_wf_phases_t *w, int phase, uint8_t *lut)
{
    uint8_t act[EPD_WF_LEVELS][EPD_WF_LEVELS];    // [from][to]
    for (int from = 0; from < EPD_WF_LEVELS; from++) {
        for (int to = 0; to < EPD_WF_LEVELS; to++) {
            act[from][to] = (uint8_t)epd_wf_action(w, phase, from, to);
        }
    }
    uint32_t right[EPD_WF_LEVELS][4];               // [from] -> 16 right-pixel codes << 2, packed
    for (int from = 0; from < EPD_WF_LEVELS; from++) {
        for (int q = 0; q < 4; q++) {
            uint32_t v = 0;
            for (int k = 0; k < 4; k++) {
                v |= (uint32_t)(act[from][q * 4 + k] << 2) << (8 * k);   // little-endian byte k
            }
            right[from][q] = v;
        }
    }
    uint32_t *out = (uint32_t *)(void *)lut;
    for (int c = 0; c < 256; c++) {
        const uint8_t *l = act[c >> 4];
        const uint32_t *r = right[c & 15];
        for (int h = 0; h < 16; h++) {
            const uint32_t base = l[h] * 0x01010101u;
            *out++ = base | r[0];
            *out++ = base | r[1];
            *out++ = base | r[2];
            *out++ = base | r[3];
        }
    }
}

bool epd_wf_phase_same(const epd_wf_phases_t *w, int a, int b)
{
    return memcmp(w->lut + (size_t)a * EPD_WF_LEVELS * 4, w->lut + (size_t)b * EPD_WF_LEVELS * 4,
                  EPD_WF_LEVELS * 4) == 0;
}

#define PAIR(lut, c, n) ((lut)[((uint32_t)(c) << 8) | (n)])

// One bus byte from two cur and two next plane bytes
static inline uint32_t byte_of(const uint8_t *lut, uint32_t c, uint32_t n)
{
    return PAIR(lut, c & 0xFF, n & 0xFF) | (PAIR(lut, (c >> 8) & 0xFF, (n >> 8) & 0xFF) << 4);
}

// Hot loop: 32-bit plane loads (8 pixels), one 32-bit bus store per 16 pixels.
// Planes and phase buffers must be 4-byte aligned at their start; rows are
// (plane stride multiple of 8, bus stride multiple of 4 for the fast path).
void epd_wf_phase(const uint8_t *lut, const uint8_t *cur, const uint8_t *next, size_t plane_stride,
                  uint8_t *out, size_t out_stride, epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t bytes = (size_t)band.w / 4;
    for (int y = band.y; y < band.y + band.h; y++) {
        const uint8_t *c = cur + (size_t)y * plane_stride + (size_t)band.x / 2;
        const uint8_t *n = next + (size_t)y * plane_stride + (size_t)band.x / 2;
        uint8_t *o = out + (size_t)y * out_stride + (size_t)band.x / 4;
        size_t i = 0;
        // head: until the bus pointer (and with it both plane pointers) is word aligned
        for (; i < bytes && (((uintptr_t)o & 3) || ((uintptr_t)c & 7) || ((uintptr_t)n & 7)); i++) {
            *o++ = (uint8_t)byte_of(lut, c[0] | (c[1] << 8), n[0] | (n[1] << 8));
            c += 2;
            n += 2;
        }
        if (!((uintptr_t)c & 3) && !((uintptr_t)n & 3)) {
            const uint32_t *cw = (const uint32_t *)(const void *)c;
            const uint32_t *nw = (const uint32_t *)(const void *)n;
            uint32_t *ow = (uint32_t *)(void *)o;
            for (; i + 4 <= bytes; i += 4) {
                const uint32_t c0 = *cw++, c1 = *cw++;
                const uint32_t n0 = *nw++, n1 = *nw++;
                *ow++ = byte_of(lut, c0, n0) | (byte_of(lut, c0 >> 16, n0 >> 16) << 8) |
                        (byte_of(lut, c1, n1) << 16) | (byte_of(lut, c1 >> 16, n1 >> 16) << 24);
            }
            c = (const uint8_t *)cw;
            n = (const uint8_t *)nw;
            o = (uint8_t *)ow;
        }
        for (; i < bytes; i++) {    // tail
            *o++ = (uint8_t)byte_of(lut, c[0] | (c[1] << 8), n[0] | (n[1] << 8));
            c += 2;
            n += 2;
        }
    }
}

void epd_wf_phase2(const uint8_t *lut_a, const uint8_t *lut_b, const uint8_t *cur, const uint8_t *next,
                   size_t plane_stride, uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t bytes = (size_t)band.w / 4;
    for (int y = band.y; y < band.y + band.h; y++) {
        const uint8_t *c = cur + (size_t)y * plane_stride + (size_t)band.x / 2;
        const uint8_t *n = next + (size_t)y * plane_stride + (size_t)band.x / 2;
        uint8_t *oa = out_a + (size_t)y * out_stride + (size_t)band.x / 4;
        uint8_t *ob = out_b + (size_t)y * out_stride + (size_t)band.x / 4;
        size_t i = 0;
        for (; i < bytes && (((uintptr_t)oa & 3) || ((uintptr_t)ob & 3) || ((uintptr_t)c & 7) || ((uintptr_t)n & 7));
             i++) {
            const uint32_t cc = c[0] | (c[1] << 8), nn = n[0] | (n[1] << 8);
            *oa++ = (uint8_t)byte_of(lut_a, cc, nn);
            *ob++ = (uint8_t)byte_of(lut_b, cc, nn);
            c += 2;
            n += 2;
        }
        if (!((uintptr_t)c & 3) && !((uintptr_t)n & 3) && !((uintptr_t)oa & 3) && !((uintptr_t)ob & 3)) {
            const uint32_t *cw = (const uint32_t *)(const void *)c;
            const uint32_t *nw = (const uint32_t *)(const void *)n;
            uint32_t *wa = (uint32_t *)(void *)oa;
            uint32_t *wb = (uint32_t *)(void *)ob;
            for (; i + 4 <= bytes; i += 4) {
                const uint32_t c0 = *cw++, c1 = *cw++;
                const uint32_t n0 = *nw++, n1 = *nw++;
                *wa++ = byte_of(lut_a, c0, n0) | (byte_of(lut_a, c0 >> 16, n0 >> 16) << 8) |
                        (byte_of(lut_a, c1, n1) << 16) | (byte_of(lut_a, c1 >> 16, n1 >> 16) << 24);
                *wb++ = byte_of(lut_b, c0, n0) | (byte_of(lut_b, c0 >> 16, n0 >> 16) << 8) |
                        (byte_of(lut_b, c1, n1) << 16) | (byte_of(lut_b, c1 >> 16, n1 >> 16) << 24);
            }
            c = (const uint8_t *)cw;
            n = (const uint8_t *)nw;
            oa = (uint8_t *)wa;
            ob = (uint8_t *)wb;
        }
        for (; i < bytes; i++) {
            const uint32_t cc = c[0] | (c[1] << 8), nn = n[0] | (n[1] << 8);
            *oa++ = (uint8_t)byte_of(lut_a, cc, nn);
            *ob++ = (uint8_t)byte_of(lut_b, cc, nn);
            c += 2;
            n += 2;
        }
    }
}

void epd_wf_pair_plane(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint16_t *pp, epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t bytes = (size_t)band.w / 2;
    for (int y = band.y; y < band.y + band.h; y++) {
        const size_t off = (size_t)y * plane_stride + (size_t)band.x / 2;
        const uint8_t *c = cur + off;
        const uint8_t *n = next + off;
        uint16_t *p = pp + off;
        for (size_t i = 0; i < bytes; i++) {
            p[i] = (uint16_t)((c[i] << 8) | n[i]);
        }
    }
}

// Two pair-plane entries (one 32-bit load) -> one bus byte
#define PP_BYTE(lut, w) ((uint32_t)(lut)[(w) & 0xFFFFu] | ((uint32_t)(lut)[(w) >> 16] << 4))

void epd_wf_phase_pp(const uint8_t *lut, const uint16_t *pp, size_t pp_stride, uint8_t *out, size_t out_stride,
                     epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t bytes = (size_t)band.w / 4;
    for (int y = band.y; y < band.y + band.h; y++) {
        const uint16_t *p = pp + (size_t)y * pp_stride + (size_t)band.x / 2;
        uint8_t *o = out + (size_t)y * out_stride + (size_t)band.x / 4;
        size_t i = 0;
        for (; i < bytes && (((uintptr_t)o & 3) || ((uintptr_t)p & 3)); i++) {
            *o++ = (uint8_t)(lut[p[0]] | (lut[p[1]] << 4));
            p += 2;
        }
        if (!((uintptr_t)p & 3)) {
            const uint32_t *pw = (const uint32_t *)(const void *)p;
            uint32_t *ow = (uint32_t *)(void *)o;
            for (; i + 4 <= bytes; i += 4) {
                const uint32_t w0 = pw[0], w1 = pw[1], w2 = pw[2], w3 = pw[3];
                pw += 4;
                *ow++ = PP_BYTE(lut, w0) | (PP_BYTE(lut, w1) << 8) | (PP_BYTE(lut, w2) << 16) |
                        (PP_BYTE(lut, w3) << 24);
            }
            p = (const uint16_t *)pw;
            o = (uint8_t *)ow;
        }
        for (; i < bytes; i++) {
            *o++ = (uint8_t)(lut[p[0]] | (lut[p[1]] << 4));
            p += 2;
        }
    }
}

void epd_wf_phase2_pp(const uint8_t *lut_a, const uint8_t *lut_b, const uint16_t *pp, size_t pp_stride,
                      uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t bytes = (size_t)band.w / 4;
    for (int y = band.y; y < band.y + band.h; y++) {
        const uint16_t *p = pp + (size_t)y * pp_stride + (size_t)band.x / 2;
        uint8_t *oa = out_a + (size_t)y * out_stride + (size_t)band.x / 4;
        uint8_t *ob = out_b + (size_t)y * out_stride + (size_t)band.x / 4;
        size_t i = 0;
        for (; i < bytes && (((uintptr_t)oa & 3) || ((uintptr_t)ob & 3) || ((uintptr_t)p & 3)); i++) {
            *oa++ = (uint8_t)(lut_a[p[0]] | (lut_a[p[1]] << 4));
            *ob++ = (uint8_t)(lut_b[p[0]] | (lut_b[p[1]] << 4));
            p += 2;
        }
        if (!((uintptr_t)p & 3) && !((uintptr_t)oa & 3) && !((uintptr_t)ob & 3)) {
            const uint32_t *pw = (const uint32_t *)(const void *)p;
            uint32_t *wa = (uint32_t *)(void *)oa;
            uint32_t *wb = (uint32_t *)(void *)ob;
            for (; i + 4 <= bytes; i += 4) {
                const uint32_t w0 = pw[0], w1 = pw[1], w2 = pw[2], w3 = pw[3];
                pw += 4;
                *wa++ = PP_BYTE(lut_a, w0) | (PP_BYTE(lut_a, w1) << 8) | (PP_BYTE(lut_a, w2) << 16) |
                        (PP_BYTE(lut_a, w3) << 24);
                *wb++ = PP_BYTE(lut_b, w0) | (PP_BYTE(lut_b, w1) << 8) | (PP_BYTE(lut_b, w2) << 16) |
                        (PP_BYTE(lut_b, w3) << 24);
            }
            p = (const uint16_t *)pw;
            oa = (uint8_t *)wa;
            ob = (uint8_t *)wb;
        }
        for (; i < bytes; i++) {
            *oa++ = (uint8_t)(lut_a[p[0]] | (lut_a[p[1]] << 4));
            *ob++ = (uint8_t)(lut_b[p[0]] | (lut_b[p[1]] << 4));
            p += 2;
        }
    }
}

void epd_wf_commit(uint8_t *cur, const uint8_t *next, size_t plane_stride, epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    for (int y = band.y; y < band.y + band.h; y++) {
        const size_t off = (size_t)y * plane_stride + (size_t)band.x / 2;
        memcpy(cur + off, next + off, (size_t)band.w / 2);
    }
}

epd_rect_t epd_wf_band_align(epd_rect_t r, uint16_t pan_w, uint16_t pan_h)
{
    int x0 = r.x < 0 ? 0 : r.x;
    int y0 = r.y < 0 ? 0 : r.y;
    int x1 = r.x + r.w > pan_w ? pan_w : r.x + r.w;
    int y1 = r.y + r.h > pan_h ? pan_h : r.y + r.h;
    if (r.w <= 0 || r.h <= 0 || x1 <= x0 || y1 <= y0) {
        return (epd_rect_t){0, 0, 0, 0};
    }
    x0 &= ~3;
    x1 = (x1 + 3) & ~3;
    if (x1 > pan_w) {
        x1 = pan_w & ~3;
    }
    return (epd_rect_t){(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0), (int16_t)(y1 - y0)};
}

const epd_wf_phases_t *epd_wf_pick(const epd_wf_set_t *s, epd_wf_kind_t k, int temp_c)
{
    if (!s || s->n == 0 || k >= EPD_WF_COUNT) {
        return NULL;
    }
    const epd_wf_range_t *best = &s->ranges[0];
    int best_d = 1 << 30;
    for (uint8_t i = 0; i < s->n; i++) {
        const epd_wf_range_t *r = &s->ranges[i];
        const int d = temp_c < r->t_min ? r->t_min - temp_c : temp_c > r->t_max ? temp_c - r->t_max : 0;
        if (d < best_d) {
            best_d = d;
            best = r;
        }
    }
    return best->w[k].lut && best->w[k].phases ? &best->w[k] : NULL;
}

epd_wf_kind_t epd_wf_kind_for(epd_mode_t m, int temp_c, bool temp_valid)
{
    const int t = temp_valid ? temp_c : EPD_WF_TEMP_DEFAULT;
    const bool fast_ok = t >= EPD_WF_TEMP_FAST_MIN && t <= EPD_WF_TEMP_FAST_MAX;
    switch (m) {
    case EPD_M_FAST:
        return fast_ok ? EPD_WF_DU_SHORT : EPD_WF_GC16;
    case EPD_M_PARTIAL:
        return fast_ok ? EPD_WF_DU : EPD_WF_GC16;
    case EPD_M_GRAY:
        return EPD_WF_GL16;
    default:
        return EPD_WF_GC16;
    }
}
