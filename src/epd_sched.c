// Refresh scheduler: see epd_sched.h. Pure C, no I/O, no panel knowledge.

#include <string.h>
#include "epd_sched.h"

// ---- Pending rect bookkeeping ----------------------------------------------
// Kept private and minimal so the controller can later swap it for epd_dirty
// without touching epd_sched.h.

static int64_t rect_area(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
    return (int64_t)(x1 - x0) * (int64_t)(y1 - y0);
}

// Clip r to the panel and align x outwards to s->align; false when empty
static bool rect_normalize(const epd_sched_t *s, epd_rect_t r, epd_rect_t *out)
{
    if (r.w <= 0 || r.h <= 0) {
        return false;
    }
    int32_t x0 = r.x, y0 = r.y, x1 = (int32_t)r.x + r.w, y1 = (int32_t)r.y + r.h;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > s->pan_w) x1 = s->pan_w;
    if (y1 > s->pan_h) y1 = s->pan_h;
    if (x0 >= x1 || y0 >= y1) {
        return false;
    }
    int32_t a = s->align;
    x0 = x0 / a * a;
    x1 = (x1 + a - 1) / a * a;
    if (x1 > s->pan_w) x1 = s->pan_w;
    out->x = (int16_t)x0;
    out->y = (int16_t)y0;
    out->w = (int16_t)(x1 - x0);
    out->h = (int16_t)(y1 - y0);
    return true;
}

static epd_rect_t rect_union(epd_rect_t a, epd_rect_t b)
{
    int32_t x0 = a.x < b.x ? a.x : b.x;
    int32_t y0 = a.y < b.y ? a.y : b.y;
    int32_t ax1 = a.x + a.w, bx1 = b.x + b.w, ay1 = a.y + a.h, by1 = b.y + b.h;
    int32_t x1 = ax1 > bx1 ? ax1 : bx1;
    int32_t y1 = ay1 > by1 ? ay1 : by1;
    epd_rect_t u = {(int16_t)x0, (int16_t)y0, (int16_t)(x1 - x0), (int16_t)(y1 - y0)};
    return u;
}

static bool rect_contains(epd_rect_t o, epd_rect_t r)
{
    return r.x >= o.x && r.y >= o.y && r.x + r.w <= o.x + o.w && r.y + r.h <= o.y + o.h;
}

static void pending_add(epd_sched_t *s, epd_rect_t raw)
{
    epd_rect_t r;
    if (!rect_normalize(s, raw, &r)) {
        return;
    }
    for (uint8_t i = 0; i < s->n; i++) {
        if (rect_contains(s->rects[i], r)) {
            return;
        }
    }
    if (s->n < EPD_SCHED_MAX_RECTS) {
        s->rects[s->n++] = r;
        return;
    }
    // Full: grow the rect whose bounding box grows the least
    uint8_t best = 0;
    int64_t best_growth = INT64_MAX;
    for (uint8_t i = 0; i < s->n; i++) {
        epd_rect_t u = rect_union(s->rects[i], r);
        int64_t growth = rect_area(u.x, u.y, u.x + u.w, u.y + u.h) -
                         rect_area(s->rects[i].x, s->rects[i].y,
                                   s->rects[i].x + s->rects[i].w, s->rects[i].y + s->rects[i].h);
        if (growth < best_growth) {
            best_growth = growth;
            best = i;
        }
    }
    s->rects[best] = rect_union(s->rects[best], r);
}

// ---- Scheduler --------------------------------------------------------------

void epd_sched_init(epd_sched_t *s, uint16_t pan_w, uint16_t pan_h, uint8_t align, uint32_t busy_timeout_ms)
{
    memset(s, 0, sizeof(*s));
    s->pan_w = pan_w;
    s->pan_h = pan_h;
    s->align = align ? align : 1;
    s->busy_timeout_ms = busy_timeout_ms;
    s->mode = EPD_M_FAST;
}

void epd_sched_frame(epd_sched_t *s, const epd_rect_t *panel_rects, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        pending_add(s, panel_rects[i]);
    }
    if (n > 0 && s->gray_ready) {
        s->gray_ready = false;   // the planes show an older image: make them again
        s->gray_asked = false;
    }
}

void epd_sched_request(epd_sched_t *s, epd_mode_t mode, uint32_t seq)
{
    if (!s->has_intent || mode > s->mode) {
        s->mode = mode;
    }
    s->has_intent = true;
    s->awaiting_frame = true;
    s->hold = false;            // the tap produced its intent: the held frames go with it
    if (seq > s->seq) {
        s->seq = seq;
    }
}

void epd_sched_frame_end(epd_sched_t *s)
{
    s->awaiting_frame = false;
}

void epd_sched_gray_ready(epd_sched_t *s)
{
    s->gray_ready = true;
}

void epd_sched_force_full(epd_sched_t *s)
{
    s->force_full = true;
}

void epd_sched_hold(epd_sched_t *s, uint32_t until_ms)
{
    s->hold = until_ms != 0;
    s->hold_until = until_ms;
}

// Wrap-safe "now >= hold_until"
static bool hold_over(const epd_sched_t *s, uint32_t now_ms)
{
    return (int32_t)(now_ms - s->hold_until) >= 0;
}

// Pending work is a run without an intent that the touch hold keeps back
static bool held(const epd_sched_t *s, uint32_t now_ms)
{
    return s->hold && !s->has_intent && !hold_over(s, now_ms);
}

void epd_sched_done(epd_sched_t *s, uint32_t now_ms)
{
    (void)now_ms;
    if (!s->busy) {
        return;   // late done for a run already abandoned by a timeout
    }
    s->busy = false;
    s->commit_due = true;
    s->commit_seq = s->run_seq;
    s->commit_error = false;
}

void epd_sched_fail(epd_sched_t *s)
{
    if (!s->busy) {
        return;
    }
    s->busy = false;
    s->reinit_due = true;
    s->commit_due = true;
    s->commit_seq = s->run_seq;
    s->commit_error = true;
    s->force_full = true;
    s->fail_pending = true;
}

static bool work_pending(const epd_sched_t *s)
{
    return s->n > 0 || s->has_intent;
}

static bool timed_out(const epd_sched_t *s, uint32_t now_ms)
{
    return (uint32_t)(now_ms - s->start_ms) >= s->busy_timeout_ms;
}

static epd_mode_t pending_mode(const epd_sched_t *s)
{
    if (s->force_full) {
        return EPD_M_FULL_CLEAN;
    }
    return s->has_intent ? s->mode : EPD_M_FAST;
}

// Pending work that cannot start until the caller acts (frame_end, gray_ready)
static bool blocked(const epd_sched_t *s)
{
    return s->awaiting_frame || (pending_mode(s) == EPD_M_GRAY && !s->gray_ready && s->gray_asked);
}

bool epd_sched_next(epd_sched_t *s, uint32_t now_ms, epd_act_t *out)
{
    memset(out, 0, sizeof(*out));
    if (s->busy && timed_out(s, now_ms)) {
        s->busy = false;
        s->reinit_due = true;
        s->commit_due = true;
        s->commit_seq = s->run_seq;
        s->commit_error = true;
        s->force_full = true;
        s->fail_pending = false;
    }
    if (s->reinit_due) {
        s->reinit_due = false;
        out->kind = EPD_ACT_REINIT;
        out->failed = s->fail_pending;
        return true;
    }
    if (s->commit_due) {
        s->commit_due = false;
        uint32_t seq = s->commit_seq;
        if (seq != 0) {
            if (seq < s->last_seq) {
                seq = s->last_seq;   // never report a seq lower than one already reported
            }
            s->last_seq = seq;
        }
        out->kind = EPD_ACT_COMMIT;
        out->seq = seq;
        out->error = s->commit_error;
        out->failed = s->fail_pending;
        s->fail_pending = false;
        return true;
    }
    if (s->hold && hold_over(s, now_ms)) {
        s->hold = false;        // expired: never compare against a stale deadline later
    }
    if (s->busy || !work_pending(s) || s->awaiting_frame || held(s, now_ms)) {
        return false;
    }

    epd_mode_t mode = pending_mode(s);
    if (mode == EPD_M_GRAY && !s->gray_ready) {
        if (s->gray_asked) {
            return false;
        }
        s->gray_asked = true;
        out->kind = EPD_ACT_NEED_GRAY;
        return true;
    }
    out->kind = EPD_ACT_START;
    out->mode = mode;
    if (mode >= EPD_M_GRAY || s->n == 0) {
        out->rects[0] = (epd_rect_t){0, 0, (int16_t)s->pan_w, (int16_t)s->pan_h};
        out->n = 1;
    } else {
        memcpy(out->rects, s->rects, sizeof(s->rects[0]) * s->n);
        out->n = s->n;
    }

    s->busy = true;
    s->start_ms = now_ms;
    s->run_seq = s->has_intent ? s->seq : 0;
    s->n = 0;
    s->has_intent = false;
    s->mode = EPD_M_FAST;
    s->seq = 0;
    s->force_full = false;
    s->gray_asked = false;
    s->gray_ready = false;
    return true;
}

uint32_t epd_sched_wait_ms(const epd_sched_t *s, uint32_t now_ms)
{
    if (s->reinit_due || s->commit_due) {
        return 0;
    }
    if (s->busy) {
        uint32_t elapsed = now_ms - s->start_ms;
        return elapsed >= s->busy_timeout_ms ? 0 : s->busy_timeout_ms - elapsed;
    }
    if (!work_pending(s) || blocked(s)) {
        return UINT32_MAX;
    }
    return held(s, now_ms) ? s->hold_until - now_ms : 0;
}

bool epd_sched_idle(const epd_sched_t *s)
{
    return !s->busy && !s->reinit_due && !s->commit_due && !work_pending(s);
}
