#include <string.h>
#include "check.h"
#include "epd_sched.h"

#define W 800
#define H 480
#define TMO 2000

static epd_rect_t R(int x, int y, int w, int h)
{
    epd_rect_t r = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
    return r;
}

static bool is_full_screen(const epd_act_t *a)
{
    return a->n == 1 && a->rects[0].x == 0 && a->rects[0].y == 0 &&
           a->rects[0].w == W && a->rects[0].h == H;
}

// true when r lies inside one of the action's rects
static bool covered(const epd_act_t *a, epd_rect_t r)
{
    for (uint8_t i = 0; i < a->n; i++) {
        const epd_rect_t *o = &a->rects[i];
        if (r.x >= o->x && r.y >= o->y && r.x + r.w <= o->x + o->w && r.y + r.h <= o->y + o->h) {
            return true;
        }
    }
    return false;
}

static void test_sched_frame_without_intent_is_fast_seq0(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    CHECK(!epd_sched_next(&s, 0, &a));
    epd_rect_t r = R(16, 20, 32, 10);
    epd_sched_frame(&s, &r, 1);
    CHECK(epd_sched_next(&s, 0, &a));
    CHECK(a.kind == EPD_ACT_START);
    CHECK(a.mode == EPD_M_FAST);
    CHECK(a.n == 1);
    CHECK(a.rects[0].x == 16 && a.rects[0].y == 20 && a.rects[0].w == 32 && a.rects[0].h == 10);
    epd_sched_done(&s, 100);
    CHECK(epd_sched_next(&s, 100, &a));
    CHECK(a.kind == EPD_ACT_COMMIT);
    CHECK(a.seq == 0);
    CHECK(!a.error);
    CHECK(!epd_sched_next(&s, 100, &a));
    CHECK(epd_sched_idle(&s));
}

static void test_sched_intent_without_frame_uses_full_screen(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_sched_request(&s, EPD_M_FULL_CLEAN, 5);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 0, &a));
    CHECK(a.kind == EPD_ACT_START);
    CHECK(a.mode == EPD_M_FULL_CLEAN);
    CHECK(is_full_screen(&a));
    epd_sched_done(&s, 10);
    CHECK(epd_sched_next(&s, 10, &a));
    CHECK(a.kind == EPD_ACT_COMMIT && a.seq == 5);
}

static void test_sched_coalesces_while_busy(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_rect_t r0 = R(0, 0, 8, 8);
    epd_sched_frame(&s, &r0, 1);
    CHECK(epd_sched_next(&s, 0, &a));
    CHECK(a.kind == EPD_ACT_START);

    epd_rect_t f[3] = {R(100, 100, 50, 20), R(400, 300, 10, 10), R(700, 0, 100, 480)};
    epd_sched_frame(&s, &f[0], 1);
    epd_sched_frame(&s, &f[1], 1);
    epd_sched_frame(&s, &f[2], 1);
    epd_sched_request(&s, EPD_M_PARTIAL, 7);
    epd_sched_frame_end(&s);
    CHECK(!epd_sched_next(&s, 50, &a));
    CHECK(!epd_sched_idle(&s));

    epd_sched_done(&s, 60);
    CHECK(epd_sched_next(&s, 60, &a));
    CHECK(a.kind == EPD_ACT_COMMIT && a.seq == 0);
    CHECK(epd_sched_next(&s, 60, &a));
    CHECK(a.kind == EPD_ACT_START);
    CHECK(a.mode == EPD_M_PARTIAL);
    CHECK(a.n >= 1 && a.n <= EPD_SCHED_MAX_RECTS);
    for (int i = 0; i < 3; i++) {
        CHECK(covered(&a, f[i]));
    }
    epd_sched_done(&s, 120);
    CHECK(epd_sched_next(&s, 120, &a));
    CHECK(a.kind == EPD_ACT_COMMIT && a.seq == 7);
    CHECK(!epd_sched_next(&s, 120, &a));
    CHECK(epd_sched_idle(&s));
}

static void test_sched_mode_upgrade_and_commit_order(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_sched_request(&s, EPD_M_PARTIAL, 3);
    epd_sched_frame_end(&s);
    epd_sched_request(&s, EPD_M_FULL_CLEAN, 4);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 0, &a));
    CHECK(a.kind == EPD_ACT_START && a.mode == EPD_M_FULL_CLEAN);
    CHECK(is_full_screen(&a));
    epd_sched_done(&s, 10);
    CHECK(epd_sched_next(&s, 10, &a));
    CHECK(a.kind == EPD_ACT_COMMIT && a.seq == 4);
    CHECK(!epd_sched_next(&s, 10, &a));

    // A stale intent arriving later must not make the commit seq go backwards
    epd_sched_request(&s, EPD_M_PARTIAL, 3);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 20, &a));
    CHECK(a.kind == EPD_ACT_START && a.mode == EPD_M_PARTIAL);
    epd_sched_done(&s, 30);
    CHECK(epd_sched_next(&s, 30, &a));
    CHECK(a.kind == EPD_ACT_COMMIT);
    CHECK(a.seq >= 4);
}

static void test_sched_invert_forces_full_clean(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_sched_force_full(&s);
    epd_rect_t r = R(8, 8, 8, 8);
    epd_sched_frame(&s, &r, 1);
    CHECK(epd_sched_next(&s, 0, &a));
    CHECK(a.kind == EPD_ACT_START && a.mode == EPD_M_FULL_CLEAN);
    CHECK(is_full_screen(&a));
    // force_full is consumed by that run
    epd_sched_done(&s, 10);
    CHECK(epd_sched_next(&s, 10, &a) && a.kind == EPD_ACT_COMMIT);
    epd_sched_frame(&s, &r, 1);
    CHECK(epd_sched_next(&s, 20, &a));
    CHECK(a.kind == EPD_ACT_START && a.mode == EPD_M_FAST && !is_full_screen(&a));
}

static void test_sched_busy_timeout_recovers(void)
{
    epd_sched_t s;
    epd_act_t a;
    const uint32_t t0 = 1000;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_sched_request(&s, EPD_M_PARTIAL, 9);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, t0, &a));
    CHECK(a.kind == EPD_ACT_START);
    CHECK(epd_sched_wait_ms(&s, t0) == TMO);
    CHECK(epd_sched_wait_ms(&s, t0 + 500) == TMO - 500);
    CHECK(!epd_sched_next(&s, t0 + 1999, &a));
    CHECK(epd_sched_next(&s, t0 + 2000, &a));
    CHECK(a.kind == EPD_ACT_REINIT);
    CHECK(epd_sched_next(&s, t0 + 2000, &a));
    CHECK(a.kind == EPD_ACT_COMMIT && a.error && a.seq == 9);
    CHECK(!epd_sched_next(&s, t0 + 2000, &a));
    // A late done for the abandoned run is ignored
    epd_sched_done(&s, t0 + 2100);
    CHECK(!epd_sched_next(&s, t0 + 2100, &a));
    epd_rect_t r = R(8, 8, 8, 8);
    epd_sched_frame(&s, &r, 1);
    CHECK(epd_sched_next(&s, t0 + 3000, &a));
    CHECK(a.kind == EPD_ACT_START && a.mode == EPD_M_FULL_CLEAN && is_full_screen(&a));
    epd_sched_done(&s, t0 + 3100);
    CHECK(epd_sched_next(&s, t0 + 3100, &a));
    CHECK(a.kind == EPD_ACT_COMMIT && !a.error);
}

static void test_sched_timeout_wraps(void)
{
    epd_sched_t s;
    epd_act_t a;
    const uint32_t t0 = UINT32_MAX - 500;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_rect_t r = R(0, 0, 8, 8);
    epd_sched_frame(&s, &r, 1);
    CHECK(epd_sched_next(&s, t0, &a) && a.kind == EPD_ACT_START);
    CHECK(!epd_sched_next(&s, t0 + 1999u, &a));               // wrapped past zero
    CHECK(epd_sched_wait_ms(&s, t0 + 1999u) == 1);
    CHECK(epd_sched_next(&s, t0 + 2000u, &a));
    CHECK(a.kind == EPD_ACT_REINIT);
}

static void test_sched_idle(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    CHECK(epd_sched_idle(&s));
    CHECK(epd_sched_wait_ms(&s, 0) == UINT32_MAX);
    epd_rect_t r = R(0, 0, 8, 8);
    epd_sched_frame(&s, &r, 1);
    CHECK(!epd_sched_idle(&s));
    CHECK(epd_sched_wait_ms(&s, 0) == 0);
    CHECK(epd_sched_next(&s, 0, &a) && a.kind == EPD_ACT_START);
    CHECK(!epd_sched_idle(&s));
    epd_sched_done(&s, 10);
    CHECK(!epd_sched_idle(&s));                                  // commit not reported yet
    CHECK(epd_sched_next(&s, 10, &a) && a.kind == EPD_ACT_COMMIT);
    CHECK(epd_sched_idle(&s));
    // Empty and off-screen frames leave it idle
    epd_rect_t junk[2] = {R(10, 10, 0, 10), R(900, 0, 10, 10)};
    epd_sched_frame(&s, junk, 2);
    CHECK(epd_sched_idle(&s));
}

static void test_sched_clip_align_and_overflow(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_rect_t r[2] = {R(-5, 10, 20, 10), R(795, 470, 20, 20)};
    epd_sched_frame(&s, r, 2);
    CHECK(epd_sched_next(&s, 0, &a) && a.kind == EPD_ACT_START);
    CHECK(a.n == 2);
    CHECK(a.rects[0].x == 0 && a.rects[0].y == 10 && a.rects[0].w == 16 && a.rects[0].h == 10);
    CHECK(a.rects[1].x == 792 && a.rects[1].y == 470 && a.rects[1].w == 8 && a.rects[1].h == 10);
    epd_sched_done(&s, 1);
    CHECK(epd_sched_next(&s, 1, &a) && a.kind == EPD_ACT_COMMIT);

    // 20 disjoint rects fit in EPD_SCHED_MAX_RECTS and stay covered
    epd_rect_t many[20];
    for (int i = 0; i < 20; i++) {
        many[i] = R((i % 5) * 160 + 3, (i / 5) * 120 + 5, 17, 9);
    }
    epd_sched_frame(&s, many, 20);
    CHECK(epd_sched_next(&s, 2, &a) && a.kind == EPD_ACT_START);
    CHECK(a.n >= 1 && a.n <= EPD_SCHED_MAX_RECTS);
    for (int i = 0; i < 20; i++) {
        CHECK(covered(&a, many[i]));
    }
    for (uint8_t i = 0; i < a.n; i++) {
        CHECK(a.rects[i].x % 8 == 0 && a.rects[i].w % 8 == 0);
        CHECK(a.rects[i].x + a.rects[i].w <= W && a.rects[i].y + a.rects[i].h <= H);
    }
}

// Review finding I1: an intent must wait for the LVGL frame that carries it.
// The caller marks the end of the render that follows the intent with
// epd_sched_frame_end(), with or without dirty areas.
static void test_sched_intent_waits_for_its_frame(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, 800, 480, 8, 2000);
    epd_sched_request(&s, EPD_M_FULL_CLEAN, 9);
    CHECK(!epd_sched_next(&s, 0, &a));          // not before the frame
    CHECK(!epd_sched_idle(&s));
    epd_sched_frame_end(&s);                    // render done, nothing dirty
    CHECK(epd_sched_next(&s, 0, &a) && a.kind == EPD_ACT_START && a.mode == EPD_M_FULL_CLEAN);

    epd_sched_init(&s, 800, 480, 8, 2000);
    const epd_rect_t r = {0, 0, 16, 8};
    epd_sched_frame(&s, &r, 1);                 // a frame without intent still runs at once
    CHECK(epd_sched_next(&s, 0, &a) && a.kind == EPD_ACT_START && a.mode == EPD_M_FAST);
    epd_sched_request(&s, EPD_M_PARTIAL, 3);    // during the run
    epd_sched_done(&s, 10);
    CHECK(epd_sched_next(&s, 10, &a) && a.kind == EPD_ACT_COMMIT);
    CHECK(!epd_sched_next(&s, 10, &a));         // the intent's frame is not rendered yet
    epd_sched_frame(&s, &r, 1);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 10, &a) && a.kind == EPD_ACT_START && a.mode == EPD_M_PARTIAL);
}

// A GRAY run needs the gray planes, which the LVGL task quantizes on demand:
// next() asks once (EPD_ACT_NEED_GRAY), then waits for epd_sched_gray_ready().
static void test_sched_gray_waits_for_planes(void)
{
    epd_sched_t s;
    epd_act_t a;
    epd_sched_init(&s, W, H, 8, TMO);
    epd_sched_request(&s, EPD_M_GRAY, 4);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 0, &a) && a.kind == EPD_ACT_NEED_GRAY);
    CHECK(!epd_sched_next(&s, 0, &a));                 // asked once only
    CHECK(epd_sched_wait_ms(&s, 0) == UINT32_MAX);     // no polling while waiting
    CHECK(!epd_sched_idle(&s));
    epd_sched_gray_ready(&s);
    CHECK(epd_sched_next(&s, 0, &a) && a.kind == EPD_ACT_START && a.mode == EPD_M_GRAY);
    CHECK(is_full_screen(&a));
    epd_sched_done(&s, 100);
    CHECK(epd_sched_next(&s, 100, &a) && a.kind == EPD_ACT_COMMIT && a.seq == 4);

    // Planes are per run: the next GRAY asks again
    epd_sched_request(&s, EPD_M_GRAY, 5);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 100, &a) && a.kind == EPD_ACT_NEED_GRAY);

    // A frame rendered after the planes were made makes them stale: ask again
    epd_sched_gray_ready(&s);
    const epd_rect_t r = R(0, 0, 16, 8);
    epd_sched_frame(&s, &r, 1);
    CHECK(epd_sched_next(&s, 100, &a) && a.kind == EPD_ACT_NEED_GRAY);

    // A FULL_CLEAN while waiting upgrades the run; no gray planes needed
    epd_sched_request(&s, EPD_M_FULL_CLEAN, 6);
    epd_sched_frame_end(&s);
    CHECK(epd_sched_next(&s, 100, &a) && a.kind == EPD_ACT_START && a.mode == EPD_M_FULL_CLEAN);
    CHECK(a.n == 1 && is_full_screen(&a));
}

int main(void)
{
    test_sched_gray_waits_for_planes();
    test_sched_intent_waits_for_its_frame();
    test_sched_frame_without_intent_is_fast_seq0();
    test_sched_intent_without_frame_uses_full_screen();
    test_sched_coalesces_while_busy();
    test_sched_mode_upgrade_and_commit_order();
    test_sched_invert_forces_full_clean();
    test_sched_busy_timeout_recovers();
    test_sched_timeout_wraps();
    test_sched_idle();
    test_sched_clip_align_and_overflow();
    TEST_DONE("sched");
}
