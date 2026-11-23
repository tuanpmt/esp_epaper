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
    uint32_t right[EPD_WF_LEVELS][4];               // [from] -> 16 right-pixel codes (bits 1:0), packed
    for (int from = 0; from < EPD_WF_LEVELS; from++) {
        for (int q = 0; q < 4; q++) {
            uint32_t v = 0;
            for (int k = 0; k < 4; k++) {
                v |= (uint32_t)act[from][q * 4 + k] << (8 * k);   // little-endian byte k
            }
            right[from][q] = v;
        }
    }
    uint32_t *out = (uint32_t *)(void *)lut;
    for (int c = 0; c < 256; c++) {
        const uint8_t *l = act[c >> 4];
        const uint32_t *r = right[c & 15];
        for (int h = 0; h < 16; h++) {
            const uint32_t base = (uint32_t)(l[h] << 2) * 0x01010101u;    // left pixel: bits 3:2
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

// One bus byte from two cur and two next plane bytes. Vendor S31 order: the
// first plane byte (pixels 0, 1) goes to the high nibble, so pixel 0 is in
// bits 7:6 and pixel 3 in bits 1:0 (epdiy_riscv_ve_stub.c LUT4 for the S31).
static inline uint32_t byte_of(const uint8_t *lut, uint32_t c, uint32_t n)
{
    return (PAIR(lut, c & 0xFF, n & 0xFF) << 4) | PAIR(lut, (c >> 8) & 0xFF, (n >> 8) & 0xFF);
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
        size_t i = 0;
        for (; i < bytes && (((uintptr_t)c & 3) || ((uintptr_t)n & 3) || ((uintptr_t)p & 7)); i++) {
            p[i] = (uint16_t)((c[i] << 8) | n[i]);
        }
        if (!((uintptr_t)(c + i) & 3) && !((uintptr_t)(n + i) & 3) && !((uintptr_t)(p + i) & 7)) {
            const uint32_t *cw = (const uint32_t *)(const void *)(c + i);
            const uint32_t *nw = (const uint32_t *)(const void *)(n + i);
            uint32_t *pw = (uint32_t *)(void *)(p + i);
            for (; i + 4 <= bytes; i += 4) {
                const uint32_t cv = *cw++, nv = *nw++;
                *pw++ = ((cv & 0xFFu) << 8) | (nv & 0xFFu) | ((cv & 0xFF00u) << 16) | ((nv & 0xFF00u) << 8);
                *pw++ = ((cv >> 8) & 0xFF00u) | ((nv >> 16) & 0xFFu) | (cv & 0xFF000000u) | ((nv >> 8) & 0xFF0000u);
            }
        }
        for (; i < bytes; i++) {
            p[i] = (uint16_t)((c[i] << 8) | n[i]);
        }
    }
}

void epd_wf_trans_plane(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint8_t *tp, epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t bytes = (size_t)band.w / 2;
    for (int y = band.y; y < band.y + band.h; y++) {
        const size_t off = (size_t)y * plane_stride + (size_t)band.x / 2;
        const uint8_t *c = cur + off;
        const uint8_t *n = next + off;
        uint8_t *t = tp + (size_t)y * plane_stride * 2 + (size_t)band.x;
        for (size_t i = 0; i < bytes; i++) {
            t[2 * i] = (uint8_t)((c[i] & 0xF0) | (n[i] >> 4));
            t[2 * i + 1] = (uint8_t)(((c[i] & 0x0F) << 4) | (n[i] & 0x0F));
        }
    }
}

void epd_wf_phase_tables(const epd_wf_phases_t *w, int phase, uint8_t t[4][256])
{
    for (int i = 0; i < 256; i++) {
        const uint8_t a = (uint8_t)epd_wf_action(w, phase, i >> 4, i & 15);
        for (int k = 0; k < 4; k++) {
            t[k][i] = (uint8_t)(a << (6 - 2 * k));    // pixel k of 4: bits 7:6 first
        }
    }
}

void epd_wf_phase_tp(const uint8_t t[4][256], const uint8_t *tp, size_t tp_stride, uint8_t *out, size_t out_stride,
                     epd_rect_t band)
{
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const uint8_t *t0 = t[0], *t1 = t[1], *t2 = t[2], *t3 = t[3];
    const size_t bytes = (size_t)band.w / 4;
    for (int y = band.y; y < band.y + band.h; y++) {
        const uint8_t *p = tp + (size_t)y * tp_stride + (size_t)band.x;
        uint8_t *o = out + (size_t)y * out_stride + (size_t)band.x / 4;
        size_t i = 0;
        for (; i < bytes && (((uintptr_t)p & 3) || ((uintptr_t)o & 3)); i++, p += 4) {
            *o++ = (uint8_t)(t0[p[0]] | t1[p[1]] | t2[p[2]] | t3[p[3]]);
        }
        if (!((uintptr_t)p & 3) && !((uintptr_t)o & 3)) {
            const uint32_t *pw = (const uint32_t *)(const void *)p;
            uint32_t *ow = (uint32_t *)(void *)o;
            for (; i + 4 <= bytes; i += 4) {
                uint32_t r = 0;
                for (int k = 0; k < 4; k++) {
                    const uint32_t v = pw[k];
                    r |= (uint32_t)(t0[v & 0xFF] | t1[(v >> 8) & 0xFF] | t2[(v >> 16) & 0xFF] | t3[v >> 24]) << (8 * k);
                }
                pw += 4;
                *ow++ = r;
            }
            p = (const uint8_t *)pw;
            o = (uint8_t *)ow;
        }
        for (; i < bytes; i++, p += 4) {
            *o++ = (uint8_t)(t0[p[0]] | t1[p[1]] | t2[p[2]] | t3[p[3]]);
        }
    }
}

// Two pair-plane entries (one 32-bit load) -> one bus byte
#define PP_BYTE(lut, w) (((uint32_t)(lut)[(w) & 0xFFFFu] << 4) | (uint32_t)(lut)[(w) >> 16])

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
            *o++ = (uint8_t)((lut[p[0]] << 4) | lut[p[1]]);
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
            *o++ = (uint8_t)((lut[p[0]] << 4) | lut[p[1]]);
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
            *oa++ = (uint8_t)((lut_a[p[0]] << 4) | lut_a[p[1]]);
            *ob++ = (uint8_t)((lut_b[p[0]] << 4) | lut_b[p[1]]);
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
            *oa++ = (uint8_t)((lut_a[p[0]] << 4) | lut_a[p[1]]);
            *ob++ = (uint8_t)((lut_b[p[0]] << 4) | lut_b[p[1]]);
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
    case EPD_M_FAST:    // stock profile 0 "Clear": DU 20 for every DU refresh (epd_refresh.cc:227-229)
        return fast_ok ? EPD_WF_DU : EPD_WF_GC16;
    case EPD_M_PARTIAL:
    case EPD_M_TRANSITION:
        return fast_ok ? EPD_WF_DU : EPD_WF_GC16;
    case EPD_M_GRAY:
        return EPD_WF_GL16;
    default:
        return EPD_WF_GC16;
    }
}

// ---- unchanged-pixel rules ----
// Own implementation of the behaviour of the vendor driver's
// MODE_SKIP_UNCHANGED LUT builders (display/output_common/lut.c:450-480,
// find_lut_functions at :568-590): only the drive decisions are mirrored.

epd_wf_rule_t epd_wf_rule_for(epd_mode_t m, epd_wf_kind_t k)
{
    // epd_refresh.cc:291-297: a full refresh, GC16 and a non-differential GL16
    // use epd_hl_update_screen_full (no skip); the rest goes through
    // epd_hl_update_area (highlevel.c:247, MODE_SKIP_UNCHANGED)
    if (m == EPD_M_FULL_CLEAN || m == EPD_M_GRAY) {
        return EPD_WF_DRIVE_ALL;
    }
    switch (k) {
    case EPD_WF_DU:
    case EPD_WF_DU_FAST:
    case EPD_WF_DU_SHORT:
        return EPD_WF_SKIP_SAME_KEEP_BLACK;
    case EPD_WF_GL16:
    case EPD_WF_GL16_TRIM:
        return EPD_WF_SKIP_SAME_GL16;
    default:
        return EPD_WF_DRIVE_ALL;
    }
}

static inline int act_of(const uint8_t d[EPD_WF_LEVELS][4], int from, int to)
{
    return (d[to][from / 4] >> (6 - 2 * (from % 4))) & 3;
}

static inline void act_set(uint8_t d[EPD_WF_LEVELS][4], int from, int to, int a)
{
    const int sh = 6 - 2 * (from % 4);
    d[to][from / 4] = (uint8_t)((d[to][from / 4] & ~(3 << sh)) | (a << sh));
}

// Phase p is one of the last four phases where the table darkens white -> black
static bool late_black_darken(const epd_wf_phases_t *w, int p)
{
    if (epd_wf_action(w, p, 15, 0) != 1) {
        return false;
    }
    int after = 0;
    for (int q = p + 1; q < w->phases; q++) {
        after += epd_wf_action(w, q, 15, 0) == 1;
    }
    return after < 4;
}

void epd_wf_phase_data(const epd_wf_phases_t *w, int phase, epd_wf_rule_t r, uint8_t d[EPD_WF_LEVELS][4])
{
    memcpy(d, w->lut + (size_t)phase * EPD_WF_LEVELS * 4, EPD_WF_LEVELS * 4);
    if (r == EPD_WF_DRIVE_ALL) {
        return;
    }
    for (int g = r == EPD_WF_SKIP_SAME_KEEP_BLACK ? 1 : 0; g < EPD_WF_LEVELS; g++) {
        act_set(d, g, g, 0);
    }
    if (r == EPD_WF_SKIP_SAME_GL16) {
        for (int f = 1; f < EPD_WF_LEVELS; f++) {
            if (act_of(d, f, 0) != 1) {
                act_set(d, f, 0, 0);    // a black target never lightens
            }
        }
        if (late_black_darken(w, phase)) {
            act_set(d, 0, 0, 1);
        }
    }
}

void epd_wf_pair_lut_rule(const epd_wf_phases_t *w, int phase, epd_wf_rule_t r, uint8_t *lut)
{
    uint8_t d[EPD_WF_LEVELS][4];
    epd_wf_phase_data(w, phase, r, d);
    const epd_wf_phases_t one = {.phases = 1, .lut = &d[0][0]};
    epd_wf_pair_lut(&one, 0, lut);
}

bool epd_wf_phase_same_rule(const epd_wf_phases_t *w, epd_wf_rule_t r, int a, int b)
{
    uint8_t da[EPD_WF_LEVELS][4], db[EPD_WF_LEVELS][4];
    epd_wf_phase_data(w, a, r, da);
    epd_wf_phase_data(w, b, r, db);
    return memcmp(da, db, sizeof(da)) == 0;
}

// Both take band.x and band.w even (whole pair-plane elements), as every
// band of the backends (multiples of 4). One pair-plane element is two
// pixels: cur byte in bits 15:8, next byte in bits 7:0, left pixel high nibble.
void epd_wf_same_hist(const uint16_t *pp, size_t pp_stride, epd_rect_t band, uint32_t hist[EPD_WF_LEVELS])
{
    memset(hist, 0, sizeof(uint32_t) * EPD_WF_LEVELS);
    if (band.w <= 0 || band.h <= 0) {
        return;
    }
    const size_t n = (size_t)band.w / 2;
    for (int y = band.y; y < band.y + band.h; y++) {
        const uint16_t *p = pp + (size_t)y * pp_stride + (size_t)band.x / 2;
        for (size_t i = 0; i < n; i++) {
            const uint32_t v = p[i];
            const uint32_t d = (v >> 8) ^ (v & 0xFF);
            if (d == 0) {           // both pixels keep their level: the common case
                hist[v >> 12]++;
                hist[(v >> 8) & 15]++;
            } else {
                if (!(d & 0xF0)) {
                    hist[v >> 12]++;
                }
                if (!(d & 0x0F)) {
                    hist[(v >> 8) & 15]++;
                }
            }
        }
    }
}

epd_rect_t epd_wf_changed_box(const uint16_t *pp, size_t pp_stride, epd_rect_t band)
{
    int x0 = INT32_MAX, y0 = -1, x1 = -1, y1 = -1;
    const int n = band.w > 0 ? band.w / 2 : 0;
    for (int y = band.y; y < band.y + band.h && n > 0; y++) {
        const uint16_t *p = pp + (size_t)y * pp_stride + (size_t)band.x / 2;
        int i = 0;
        while (i < n && (p[i] >> 8) == (p[i] & 0xFF)) {
            i++;
        }
        if (i == n) {
            continue;   // row unchanged
        }
        int k = n - 1;
        while ((p[k] >> 8) == (p[k] & 0xFF)) {
            k--;
        }
        const int l = band.x + 2 * i + (((p[i] >> 8) ^ p[i]) & 0xF0 ? 0 : 1);
        const int r = band.x + 2 * k + (((p[k] >> 8) ^ p[k]) & 0x0F ? 1 : 0);
        x0 = l < x0 ? l : x0;
        x1 = r > x1 ? r : x1;
        y0 = y0 < 0 ? y : y0;
        y1 = y;
    }
    if (x1 < 0) {
        return (epd_rect_t){band.x, band.y, 0, 0};
    }
    return (epd_rect_t){(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0 + 1), (int16_t)(y1 - y0 + 1)};
}

void epd_wf_job_begin(epd_wf_job_t *j, const epd_wf_phases_t *w, epd_wf_rule_t rule, uint8_t *lut,
                      const uint16_t *pp, size_t pp_stride, epd_rect_t band, epd_wf_pp_kernel_t kernel)
{
    memset(j, 0, sizeof(*j));
    j->w = w;
    j->rule = rule;
    j->lut = lut;
    j->pp = pp;
    j->pp_stride = pp_stride;
    j->band = band;
    j->kernel = kernel;
    j->lut_phase = -1;
    epd_wf_same_hist(pp, pp_stride, band, j->hist);
}

bool epd_wf_job_phase(epd_wf_job_t *j, int phase, uint8_t *out, size_t out_stride)
{
    const bool reuse = phase > 0 && epd_wf_phase_same_rule(j->w, j->rule, phase, phase - 1);
    if (!reuse) {
        if (j->lut_phase < 0 || !epd_wf_phase_same_rule(j->w, j->rule, phase, j->lut_phase)) {
            epd_wf_pair_lut_rule(j->w, phase, j->rule, j->lut);
            j->lut_phase = phase;
        }
        // Read back from the LUT the kernel uses: entry (g g, g g) holds the
        // code of a pixel that stays at level g
        j->phase_idle = 0;
        j->phase_hold = j->lut[0] & 3 ? j->hist[0] : 0;
        for (int g = 1; g < EPD_WF_LEVELS; g++) {
            const uint32_t b = (uint32_t)g * 0x11u;
            if (j->lut[(b << 8) | b] & 3) {
                j->phase_idle += j->hist[g];
            }
        }
        j->kernel(j->lut, j->pp, j->pp_stride, out, out_stride, j->band);
    }
    j->idle_drive += j->phase_idle;
    j->black_hold += j->phase_hold;
    return !reuse;
}
