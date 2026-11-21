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

void epd_wf_pair_lut(const epd_wf_phases_t *w, int phase, uint8_t *lut)
{
    uint8_t act[EPD_WF_LEVELS][EPD_WF_LEVELS];    // [from][to]
    for (int from = 0; from < EPD_WF_LEVELS; from++) {
        for (int to = 0; to < EPD_WF_LEVELS; to++) {
            act[from][to] = (uint8_t)epd_wf_action(w, phase, from, to);
        }
    }
    for (int c = 0; c < 256; c++) {
        const uint8_t *l = act[c >> 4];
        const uint8_t *r = act[c & 15];
        uint8_t *row = lut + ((size_t)c << 8);
        for (int n = 0; n < 256; n++) {
            row[n] = (uint8_t)(l[n >> 4] | (r[n & 15] << 2));
        }
    }
}

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
        for (size_t i = 0; i < bytes; i++) {
            o[i] = (uint8_t)(lut[((size_t)c[0] << 8) | n[0]] | (lut[((size_t)c[1] << 8) | n[1]] << 4));
            c += 2;
            n += 2;
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
