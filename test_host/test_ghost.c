#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "check.h"
#include "epd_ghost.h"

static epd_rect_t R(int x, int y, int w, int h)
{
    epd_rect_t r = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h};
    return r;
}

static uint16_t tile(const epd_ghost_t *g, int col, int row)
{
    return g->debt[row * g->cols + col];
}

// Number of tiles with a non-zero debt
static int nonzero(const epd_ghost_t *g)
{
    int n = 0;
    for (int i = 0; i < g->cols * g->rows; i++) {
        n += g->debt[i] != 0;
    }
    return n;
}

static void test_ghost_partial_tile_coverage(void)
{
    epd_ghost_t g;

    epd_ghost_init(&g, 800, 480);
    // half a tile -> half the weight
    epd_ghost_add(&g, R(0, 0, 64, 32), 16);
    CHECK(tile(&g, 0, 0) == 8);
    CHECK(nonzero(&g) == 1);
    // a whole tile -> the whole weight
    epd_ghost_add(&g, R(64, 0, 64, 64), 16);
    CHECK(tile(&g, 1, 0) == 16);
    // a tile-sized rect across four tiles -> a quarter each
    epd_ghost_add(&g, R(160, 160, 64, 64), 16);
    CHECK(tile(&g, 2, 2) == 4 && tile(&g, 3, 2) == 4);
    CHECK(tile(&g, 2, 3) == 4 && tile(&g, 3, 3) == 4);
    CHECK(nonzero(&g) == 6);
    // any driven pixel costs at least 1 (rounded up)
    epd_ghost_add(&g, R(700, 400, 1, 1), 16);
    CHECK(tile(&g, 10, 6) == 1);
    // 3/4 of a tile at weight 24 -> 18
    epd_ghost_add(&g, R(0, 64, 48, 64), 24);
    CHECK(tile(&g, 0, 1) == 18);
    // zero weight adds nothing
    epd_ghost_add(&g, R(0, 128, 64, 64), 0);
    CHECK(tile(&g, 0, 2) == 0);
}

static void test_ghost_edges_clip(void)
{
    epd_ghost_t g;

    epd_ghost_init(&g, 800, 480);
    // a quarter of the rect is on the panel
    epd_ghost_add(&g, R(-32, -32, 64, 64), 16);
    CHECK(tile(&g, 0, 0) == 4);
    CHECK(nonzero(&g) == 1);
    // bottom-right edge tile is 32 x 32: 20 x 20 driven -> ceil(16 * 400 / 1024) = 7
    epd_ghost_add(&g, R(780, 460, 100, 100), 16);
    CHECK(tile(&g, 12, 7) == 7);
    CHECK(nonzero(&g) == 2);
    // off-panel and empty rects add nothing
    epd_ghost_add(&g, R(800, 0, 10, 10), 16);
    epd_ghost_add(&g, R(0, 480, 10, 10), 16);
    epd_ghost_add(&g, R(-10, 0, 10, 10), 16);
    epd_ghost_add(&g, R(10, 10, 0, 10), 16);
    epd_ghost_add(&g, R(10, 10, 10, -5), 16);
    epd_ghost_add(&g, R(-32768, -32768, 32767, 32767), 16);
    CHECK(nonzero(&g) == 2);
    // a rect larger than the panel drives every tile fully
    epd_ghost_add(&g, R(-1000, -1000, 32767, 32767), 16);
    CHECK(tile(&g, 5, 5) == 16);
    CHECK(tile(&g, 12, 7) == 7 + 16);
    CHECK(nonzero(&g) == 13 * 8);
}

static void test_ghost_saturates(void)
{
    epd_ghost_t g;

    epd_ghost_init(&g, 800, 480);
    epd_ghost_add(&g, R(0, 0, 64, 64), 60000);
    epd_ghost_add(&g, R(0, 0, 64, 64), 60000);
    CHECK(tile(&g, 0, 0) == UINT16_MAX);
    epd_ghost_add(&g, R(0, 0, 64, 64), UINT16_MAX);
    CHECK(tile(&g, 0, 0) == UINT16_MAX);
    CHECK(epd_ghost_max(&g) == UINT16_MAX);
    // max weight over a full tile does not overflow the intermediate product
    epd_ghost_add(&g, R(64, 0, 64, 64), UINT16_MAX);
    CHECK(tile(&g, 1, 0) == UINT16_MAX);
    // many small adds saturate too
    for (int i = 0; i < 10000; i++) {
        epd_ghost_add(&g, R(128, 0, 64, 64), 32);
    }
    CHECK(tile(&g, 2, 0) == UINT16_MAX);
}

static void test_ghost_full_clean_resets(void)
{
    epd_ghost_t g;

    epd_ghost_init(&g, 1216, 684);
    epd_ghost_set_budget(&g, 192, 384);
    epd_ghost_add(&g, R(0, 0, 1216, 684), UINT16_MAX);
    epd_ghost_add(&g, R(100, 100, 300, 200), 32);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_URGENT);
    epd_ghost_clean(&g);
    CHECK(nonzero(&g) == 0);
    CHECK(epd_ghost_max(&g) == 0);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_OK);
    // grid and thresholds survive a clean
    CHECK(g.cols == 19 && g.rows == 11);
    CHECK(g.due == 192 && g.urgent == 384);
}

static void test_ghost_halve(void)
{
    epd_ghost_t g;

    epd_ghost_init(&g, 800, 480);
    epd_ghost_add(&g, R(0, 0, 64, 64), 16);
    epd_ghost_add(&g, R(64, 0, 64, 64), 7);
    epd_ghost_add(&g, R(128, 0, 64, 64), 1);
    epd_ghost_add(&g, R(192, 0, 64, 64), UINT16_MAX);
    epd_ghost_halve(&g);
    CHECK(tile(&g, 0, 0) == 8);
    CHECK(tile(&g, 1, 0) == 3);
    CHECK(tile(&g, 2, 0) == 0);
    CHECK(tile(&g, 3, 0) == UINT16_MAX / 2);
    CHECK(nonzero(&g) == 3);
}

static void test_ghost_levels(void)
{
    epd_ghost_t g;

    epd_ghost_init(&g, 800, 480);
    // init: thresholds 0 -> never, whatever the debt
    CHECK(g.due == 0 && g.urgent == 0);
    epd_ghost_add(&g, R(0, 0, 64, 64), UINT16_MAX);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_OK);

    // below, at and above each threshold; the worst tile decides, not the sum
    epd_ghost_init(&g, 800, 480);
    epd_ghost_set_budget(&g, 192, 384);
    epd_ghost_add(&g, R(0, 0, 800, 480), 191);
    CHECK(epd_ghost_max(&g) == 191);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_OK);
    epd_ghost_add(&g, R(64, 64, 64, 64), 1);
    CHECK(epd_ghost_max(&g) == 192);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_DUE);
    epd_ghost_add(&g, R(64, 64, 64, 64), 191);
    CHECK(epd_ghost_max(&g) == 383);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_DUE);
    epd_ghost_add(&g, R(64, 64, 64, 64), 1);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_URGENT);
    epd_ghost_add(&g, R(64, 64, 64, 64), 100);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_URGENT);

    // due disabled: only urgent fires
    epd_ghost_set_budget(&g, 0, 384);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_URGENT);
    epd_ghost_clean(&g);
    epd_ghost_add(&g, R(0, 0, 64, 64), 300);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_OK);

    // urgent disabled: due fires, never urgent
    epd_ghost_set_budget(&g, 192, 0);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_DUE);
    epd_ghost_add(&g, R(0, 0, 64, 64), UINT16_MAX);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_DUE);

    // both disabled ("never")
    epd_ghost_set_budget(&g, 0, 0);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_OK);
}

static void test_ghost_panel_sizes(void)
{
    epd_ghost_t g;

    // Plus: 1216 x 684 -> 19 x 11, the bottom row is 684 - 640 = 44 px tall
    epd_ghost_init(&g, 1216, 684);
    CHECK(g.cols == 19 && g.rows == 11);
    epd_ghost_add(&g, R(0, 0, 1216, 684), 16);
    CHECK(nonzero(&g) == 19 * 11);
    CHECK(tile(&g, 0, 0) == 16 && tile(&g, 18, 10) == 16 && tile(&g, 0, 10) == 16);
    // a whole bottom-row tile is 64 x 44
    epd_ghost_add(&g, R(0, 640, 64, 44), 16);
    CHECK(tile(&g, 0, 10) == 32);
    // half of it (64 x 22) -> half the weight
    epd_ghost_add(&g, R(64, 640, 64, 22), 16);
    CHECK(tile(&g, 1, 10) == 24);

    // 3.97": 800 x 480 -> 13 x 8, the last column and row are 32 px
    epd_ghost_init(&g, 800, 480);
    CHECK(g.cols == 13 && g.rows == 8);
    epd_ghost_add(&g, R(0, 0, 800, 480), 16);
    CHECK(nonzero(&g) == 13 * 8);
    CHECK(tile(&g, 12, 7) == 16 && tile(&g, 12, 0) == 16 && tile(&g, 0, 7) == 16);
    epd_ghost_add(&g, R(768, 0, 16, 64), 16);
    CHECK(tile(&g, 12, 0) == 24);

    // portrait and exact multiples
    epd_ghost_init(&g, 480, 800);
    CHECK(g.cols == 8 && g.rows == 13);
    epd_ghost_init(&g, 128, 64);
    CHECK(g.cols == 2 && g.rows == 1);

    // too many tiles -> inert grid, never out of bounds
    epd_ghost_init(&g, 2048, 2048);
    CHECK(g.cols * g.rows == 0);
    epd_ghost_set_budget(&g, 1, 2);
    epd_ghost_add(&g, R(0, 0, 2048, 2048), 16);
    CHECK(epd_ghost_max(&g) == 0);
    CHECK(epd_ghost_level(&g) == EPD_GHOST_OK);
    epd_ghost_halve(&g);
    epd_ghost_clean(&g);

    // empty panel
    epd_ghost_init(&g, 0, 0);
    CHECK(g.cols * g.rows == 0);
    epd_ghost_add(&g, R(0, 0, 10, 10), 16);
    CHECK(epd_ghost_max(&g) == 0);
}

int main(void)
{
    test_ghost_partial_tile_coverage();
    test_ghost_edges_clip();
    test_ghost_saturates();
    test_ghost_full_clean_resets();
    test_ghost_halve();
    test_ghost_levels();
    test_ghost_panel_sizes();
    TEST_DONE("ghost");
}
