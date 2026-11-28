#include "epd_ghost.h"

#include <string.h>

static uint16_t tiles_for(uint16_t px)
{
    return (uint16_t)((px + EPD_GHOST_TILE - 1) / EPD_GHOST_TILE);
}

void epd_ghost_init(epd_ghost_t *g, uint16_t pan_w, uint16_t pan_h)
{
    memset(g, 0, sizeof(*g));
    uint32_t cols = tiles_for(pan_w);
    uint32_t rows = tiles_for(pan_h);
    if (cols * rows > EPD_GHOST_MAX_TILES) {
        return;  // inert grid: cols = rows = 0
    }
    g->cols = (uint16_t)cols;
    g->rows = (uint16_t)rows;
    g->pan_w = pan_w;
    g->pan_h = pan_h;
}

void epd_ghost_set_budget(epd_ghost_t *g, uint16_t due, uint16_t urgent)
{
    g->due = due;
    g->urgent = urgent;
}

void epd_ghost_add(epd_ghost_t *g, epd_rect_t driven, uint16_t weight16)
{
    if (weight16 == 0 || g->cols == 0 || g->rows == 0 || driven.w <= 0 || driven.h <= 0) {
        return;
    }
    // clip in 32 bits: x + w may not fit in int16_t
    int32_t x0 = driven.x, y0 = driven.y;
    int32_t x1 = x0 + driven.w, y1 = y0 + driven.h;
    if (x0 < 0) {
        x0 = 0;
    }
    if (y0 < 0) {
        y0 = 0;
    }
    if (x1 > g->pan_w) {
        x1 = g->pan_w;
    }
    if (y1 > g->pan_h) {
        y1 = g->pan_h;
    }
    if (x0 >= x1 || y0 >= y1) {
        return;
    }
    for (int32_t ty = y0 / EPD_GHOST_TILE; ty * EPD_GHOST_TILE < y1; ty++) {
        int32_t t_y0 = ty * EPD_GHOST_TILE;
        int32_t t_y1 = t_y0 + EPD_GHOST_TILE < g->pan_h ? t_y0 + EPD_GHOST_TILE : g->pan_h;
        int32_t oy = (y1 < t_y1 ? y1 : t_y1) - (y0 > t_y0 ? y0 : t_y0);
        for (int32_t tx = x0 / EPD_GHOST_TILE; tx * EPD_GHOST_TILE < x1; tx++) {
            int32_t t_x0 = tx * EPD_GHOST_TILE;
            int32_t t_x1 = t_x0 + EPD_GHOST_TILE < g->pan_w ? t_x0 + EPD_GHOST_TILE : g->pan_w;
            int32_t ox = (x1 < t_x1 ? x1 : t_x1) - (x0 > t_x0 ? x0 : t_x0);
            // tile area <= 4096 and weight <= 65535: the product fits in 32 bits
            uint32_t area = (uint32_t)((t_x1 - t_x0) * (t_y1 - t_y0));
            uint32_t cost = ((uint32_t)weight16 * (uint32_t)(ox * oy) + area - 1) / area;
            uint16_t *d = &g->debt[ty * g->cols + tx];
            uint32_t sum = (uint32_t)*d + cost;
            *d = sum > UINT16_MAX ? UINT16_MAX : (uint16_t)sum;
        }
    }
}

void epd_ghost_clean(epd_ghost_t *g)
{
    memset(g->debt, 0, sizeof(g->debt));
}

void epd_ghost_halve(epd_ghost_t *g)
{
    for (uint32_t i = 0; i < (uint32_t)g->cols * g->rows; i++) {
        g->debt[i] >>= 1;
    }
}

uint16_t epd_ghost_max(const epd_ghost_t *g)
{
    uint16_t m = 0;
    for (uint32_t i = 0; i < (uint32_t)g->cols * g->rows; i++) {
        if (g->debt[i] > m) {
            m = g->debt[i];
        }
    }
    return m;
}

epd_ghost_level_t epd_ghost_level(const epd_ghost_t *g)
{
    uint16_t m = epd_ghost_max(g);
    if (g->urgent != 0 && m >= g->urgent) {
        return EPD_GHOST_URGENT;
    }
    if (g->due != 0 && m >= g->due) {
        return EPD_GHOST_DUE;
    }
    return EPD_GHOST_OK;
}
