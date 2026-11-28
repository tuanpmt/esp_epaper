#include "epd_dirty.h"

#include <string.h>

// Half-open box [x0, x1) x [y0, y1) in int32 so int16 inputs cannot overflow
typedef struct {
    int32_t x0, y0, x1, y1;
} box_t;

static int64_t box_area(box_t b)
{
    return (int64_t)(b.x1 - b.x0) * (b.y1 - b.y0);
}

static box_t box_union(box_t a, box_t b)
{
    box_t u = {
        a.x0 < b.x0 ? a.x0 : b.x0, a.y0 < b.y0 ? a.y0 : b.y0,
        a.x1 > b.x1 ? a.x1 : b.x1, a.y1 > b.y1 ? a.y1 : b.y1,
    };
    return u;
}

// Overlapping or sharing an edge segment (corner-only contact does not count)
static bool box_touch(box_t a, box_t b)
{
    int32_t ox = (a.x1 < b.x1 ? a.x1 : b.x1) - (a.x0 > b.x0 ? a.x0 : b.x0);
    int32_t oy = (a.y1 < b.y1 ? a.y1 : b.y1) - (a.y0 > b.y0 ? a.y0 : b.y0);
    return (ox >= 0 && oy > 0) || (ox > 0 && oy >= 0);
}

static box_t to_box(epd_rect_t r)
{
    box_t b = {r.x, r.y, (int32_t)r.x + r.w, (int32_t)r.y + r.h};
    return b;
}

static epd_rect_t to_rect(box_t b)
{
    epd_rect_t r = {(int16_t)b.x0, (int16_t)b.y0, (int16_t)(b.x1 - b.x0), (int16_t)(b.y1 - b.y0)};
    return r;
}

static void remove_at(box_t *b, uint8_t *n, uint8_t i)
{
    b[i] = b[*n - 1];
    (*n)--;
}

// Merge touching boxes until no pair touches; afterwards boxes are disjoint
static void coalesce(box_t *b, uint8_t *n)
{
    bool merged = true;
    while (merged) {
        merged = false;
        for (uint8_t i = 0; i < *n && !merged; i++) {
            for (uint8_t j = i + 1; j < *n && !merged; j++) {
                if (box_touch(b[i], b[j])) {
                    b[i] = box_union(b[i], b[j]);
                    remove_at(b, n, j);
                    merged = true;
                }
            }
        }
    }
}

// Merge the pair whose bounding box grows the covered area least, until n <= max
static void reduce(box_t *b, uint8_t *n, uint8_t max)
{
    while (*n > max && *n > 1) {
        uint8_t bi = 0, bj = 1;
        int64_t best = INT64_MAX;
        for (uint8_t i = 0; i < *n; i++) {
            for (uint8_t j = i + 1; j < *n; j++) {
                int64_t grow = box_area(box_union(b[i], b[j])) - box_area(b[i]) - box_area(b[j]);
                if (grow < best) {
                    best = grow;
                    bi = i;
                    bj = j;
                }
            }
        }
        b[bi] = box_union(b[bi], b[bj]);
        remove_at(b, n, bj);
        coalesce(b, n);
    }
}

static void load(const epd_dirty_t *d, box_t *b)
{
    for (uint8_t i = 0; i < d->n; i++) {
        b[i] = to_box(d->r[i]);
    }
}

static void store(epd_dirty_t *d, const box_t *b, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        d->r[i] = to_rect(b[i]);
    }
    d->n = n;
}

void epd_dirty_init(epd_dirty_t *d, uint16_t pan_w, uint16_t pan_h, uint8_t align, uint32_t full_area_pct)
{
    memset(d, 0, sizeof(*d));
    d->pan_w = pan_w;
    d->pan_h = pan_h;
    d->align = align ? align : 1;
    d->full_area_pct = full_area_pct;
}

void epd_dirty_add_all(epd_dirty_t *d)
{
    epd_rect_t full = {0, 0, (int16_t)d->pan_w, (int16_t)d->pan_h};
    d->r[0] = full;
    d->n = (d->pan_w && d->pan_h) ? 1 : 0;
}

bool epd_dirty_is_full(const epd_dirty_t *d)
{
    return d->n == 1 && d->r[0].x == 0 && d->r[0].y == 0 &&
           d->r[0].w == (int16_t)d->pan_w && d->r[0].h == (int16_t)d->pan_h;
}

void epd_dirty_add(epd_dirty_t *d, epd_rect_t panel_rect)
{
    if (panel_rect.w <= 0 || panel_rect.h <= 0 || epd_dirty_is_full(d)) {
        return;
    }
    box_t nb = to_box(panel_rect);
    int32_t a = d->align ? d->align : 1;
    if (nb.x0 < 0) nb.x0 = 0;
    if (nb.y0 < 0) nb.y0 = 0;
    if (nb.x1 > d->pan_w) nb.x1 = d->pan_w;
    if (nb.y1 > d->pan_h) nb.y1 = d->pan_h;
    if (nb.x0 >= nb.x1 || nb.y0 >= nb.y1) {
        return;
    }
    nb.x0 = nb.x0 / a * a;
    nb.x1 = (nb.x1 + a - 1) / a * a;
    if (nb.x1 > d->pan_w) nb.x1 = d->pan_w;

    box_t b[EPD_DIRTY_MAX + 1];
    uint8_t n = d->n;
    load(d, b);
    b[n++] = nb;
    coalesce(b, &n);
    reduce(b, &n, EPD_DIRTY_MAX);

    // Boxes are disjoint after coalesce, so the sum is the exact union area
    int64_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += box_area(b[i]);
    }
    if (sum * 100 > (int64_t)d->full_area_pct * d->pan_w * d->pan_h) {
        epd_dirty_add_all(d);
        return;
    }
    store(d, b, n);
}

uint8_t epd_dirty_take(epd_dirty_t *d, epd_rect_t *out, uint8_t max)
{
    if (max == 0 || out == NULL) {
        return 0;
    }
    box_t b[EPD_DIRTY_MAX + 1];
    uint8_t n = d->n;
    load(d, b);
    reduce(b, &n, max);
    for (uint8_t i = 0; i < n; i++) {
        out[i] = to_rect(b[i]);
    }
    d->n = 0;
    return n;
}
