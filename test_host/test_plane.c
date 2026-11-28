#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "check.h"
#include "epd_plane.h"

#define W 800
#define H 480
#define STRIDE1 (W / 8)
#define STRIDE4 (W / 2)

static epd_rect_t R(int x, int y, int w, int h)
{
    epd_rect_t r = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
    return r;
}

// 1 bpp, MSB first
static void flip1(uint8_t *p, int x, int y)
{
    p[(size_t)y * STRIDE1 + (size_t)x / 8] ^= (uint8_t)(0x80u >> (x & 7));
}

// 4 bpp, high nibble = left pixel
static void set4(uint8_t *p, int x, int y, uint8_t v)
{
    uint8_t *b = &p[(size_t)y * STRIDE4 + (size_t)x / 2];
    *b = (x & 1) ? (uint8_t)((*b & 0xF0) | v) : (uint8_t)((*b & 0x0F) | (v << 4));
}

static uint8_t A1[STRIDE1 * H], B1[STRIDE1 * H];
static uint8_t A4[STRIDE4 * H], B4[STRIDE4 * H];

static void test_plane_same(void)
{
    memset(A1, 0xFF, sizeof(A1));
    memcpy(B1, A1, sizeof(A1));
    const epd_rect_t all = R(0, 0, W, H);
    CHECK(!epd_plane_differs(A1, B1, STRIDE1, 1, &all, 1));
    CHECK(!epd_plane_differs(A1, B1, STRIDE1, 1, NULL, 0));
}

static void test_plane_differs_inside_rect(void)
{
    memset(A1, 0xFF, sizeof(A1));
    memcpy(B1, A1, sizeof(A1));
    flip1(B1, 100, 200);
    const epd_rect_t r[2] = {R(0, 0, 64, 64), R(96, 192, 16, 16)};
    CHECK(epd_plane_differs(A1, B1, STRIDE1, 1, r, 2));
    CHECK(!epd_plane_differs(A1, B1, STRIDE1, 1, r, 1));
    // the corners of a rect count
    memcpy(B1, A1, sizeof(A1));
    flip1(B1, 799, 479);
    const epd_rect_t all = R(0, 0, W, H);
    CHECK(epd_plane_differs(A1, B1, STRIDE1, 1, &all, 1));
    memcpy(B1, A1, sizeof(A1));
    flip1(B1, 0, 0);
    CHECK(epd_plane_differs(A1, B1, STRIDE1, 1, &all, 1));
}

static void test_plane_ignores_outside_rect(void)
{
    memset(A1, 0x00, sizeof(A1));
    memcpy(B1, A1, sizeof(A1));
    // unaligned rect x 13..18, y 10..11: pixels just outside it change
    flip1(B1, 12, 10);
    flip1(B1, 19, 10);
    flip1(B1, 15, 9);
    flip1(B1, 15, 12);
    const epd_rect_t r = R(13, 10, 6, 2);
    CHECK(!epd_plane_differs(A1, B1, STRIDE1, 1, &r, 1));
    flip1(B1, 13, 11);      // first pixel of the rect
    CHECK(epd_plane_differs(A1, B1, STRIDE1, 1, &r, 1));
    flip1(B1, 13, 11);
    flip1(B1, 18, 10);      // last pixel of the rect
    CHECK(epd_plane_differs(A1, B1, STRIDE1, 1, &r, 1));
    // an empty rect compares nothing
    const epd_rect_t e = R(0, 0, 0, 10);
    memset(B1, 0xFF, sizeof(B1));
    CHECK(!epd_plane_differs(A1, B1, STRIDE1, 1, &e, 1));
}

static void test_plane_bpp4(void)
{
    memset(A4, 0xFF, sizeof(A4));
    memcpy(B4, A4, sizeof(A4));
    set4(B4, 21, 5, 7);     // odd pixel: low nibble
    const epd_rect_t in = R(20, 4, 4, 4);
    const epd_rect_t left = R(16, 4, 5, 4);     // ends at pixel 20, shares the byte
    const epd_rect_t right = R(22, 4, 10, 4);   // starts at pixel 22
    CHECK(epd_plane_differs(A4, B4, STRIDE4, 4, &in, 1));
    CHECK(!epd_plane_differs(A4, B4, STRIDE4, 4, &left, 1));
    CHECK(!epd_plane_differs(A4, B4, STRIDE4, 4, &right, 1));
    set4(B4, 21, 5, 15);
    set4(B4, 20, 5, 3);     // even pixel: high nibble
    CHECK(epd_plane_differs(A4, B4, STRIDE4, 4, &left, 1));
    CHECK(!epd_plane_differs(A4, B4, STRIDE4, 4, &right, 1));
}

static void test_plane_sent_diff_mask(void)
{
    static uint8_t buf[STRIDE1 * H];
    epd_plane_sent_t s;
    memset(A1, 0xFF, sizeof(A1));
    epd_plane_sent_init(&s, buf, W, H, STRIDE1, 1);
    const epd_rect_t r[3] = {R(0, 0, 64, 64), R(200, 200, 64, 64), R(400, 0, 64, 64)};
    CHECK(epd_plane_sent_diff(&s, A1, r, 3) == 0x07);   // nothing sent yet: all differ
    CHECK(epd_plane_sent_diff(&s, A1, NULL, 0) == 0x01);
    CHECK(epd_plane_sent_take(&s, A1) == (uint32_t)W * H);
    CHECK(epd_plane_sent_diff(&s, A1, r, 3) == 0);
    CHECK(epd_plane_sent_diff(&s, A1, NULL, 0) == 0);    // whole plane, same
    flip1(A1, 210, 220);
    CHECK(epd_plane_sent_diff(&s, A1, r, 3) == 0x02);
    CHECK(epd_plane_sent_diff(&s, A1, NULL, 0) == 0x01);
    flip1(A1, 410, 10);
    flip1(A1, 411, 10);
    CHECK(epd_plane_sent_diff(&s, A1, r, 3) == 0x06);
    CHECK(epd_plane_sent_take(&s, A1) == 3);              // changed pixels since the last take
    epd_plane_sent_invalidate(&s);
    CHECK(epd_plane_sent_diff(&s, A1, r, 3) == 0x07);
}

// The renderer keeps drawing into the live plane while the panel refreshes:
// the copy stays what was sent, so the next run sees the change, and a run
// whose image was already sent is skipped.
static void test_plane_skip_keeps_sent_copy(void)
{
    static uint8_t buf[STRIDE1 * H];
    epd_plane_sent_t s;
    memset(A1, 0xFF, sizeof(A1));
    epd_plane_sent_init(&s, buf, W, H, STRIDE1, 1);
    (void)epd_plane_sent_take(&s, A1);               // frame A sent
    const epd_rect_t clock = R(64, 0, 128, 32);
    flip1(A1, 100, 10);                              // drawn during the BUSY time
    CHECK(s.buf[10 * STRIDE1 + 100 / 8] == 0xFF);    // the copy is still frame A
    CHECK(epd_plane_sent_diff(&s, A1, &clock, 1) == 0x01);
    flip1(A1, 100, 10);                              // drawn back: the panel already shows it
    CHECK(epd_plane_sent_diff(&s, A1, &clock, 1) == 0);
    // a run of the same image twice: the second one is a skip
    flip1(A1, 120, 20);
    CHECK(epd_plane_sent_take(&s, A1) == 1);
    CHECK(epd_plane_sent_diff(&s, A1, &clock, 1) == 0);
    CHECK(memcmp(s.buf, A1, sizeof(A1)) == 0);
}

static void test_plane_sent_take_bpp4(void)
{
    static uint8_t buf[STRIDE4 * H];
    epd_plane_sent_t s;
    memset(A4, 0x00, sizeof(A4));
    epd_plane_sent_init(&s, buf, W, H, STRIDE4, 4);
    CHECK(epd_plane_sent_take(&s, A4) == (uint32_t)W * H);
    set4(A4, 3, 3, 1);
    set4(A4, 4, 3, 2);
    set4(A4, 5, 3, 9);
    CHECK(epd_plane_sent_take(&s, A4) == 3);
}

int main(void)
{
    test_plane_same();
    test_plane_differs_inside_rect();
    test_plane_ignores_outside_rect();
    test_plane_bpp4();
    test_plane_sent_diff_mask();
    test_plane_skip_keeps_sent_copy();
    test_plane_sent_take_bpp4();
    TEST_DONE("plane");
}
