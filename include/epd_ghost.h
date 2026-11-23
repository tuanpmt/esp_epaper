#pragma once

// Ghosting debt per panel tile. A refresh adds weight x (covered fraction of
// the tile) to each tile it drove; a full clean clears everything; a whole-
// panel GL16 halves it. Pure C, no I/O: host-testable.
//
// Units: debt and thresholds are in 1/16 of one DU-equivalent pass over a
// whole tile. Rects are in panel coordinates (after rotation).

#include <stdint.h>

#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_GHOST_TILE 64            // panel pixels per tile side
#define EPD_GHOST_MAX_TILES 256      // 1216x684 -> 19 x 11 = 209; 800x480 -> 13 x 8 = 104

typedef enum { EPD_GHOST_OK, EPD_GHOST_DUE, EPD_GHOST_URGENT } epd_ghost_level_t;

typedef struct {
    uint16_t cols, rows;
    uint16_t debt[EPD_GHOST_MAX_TILES];  // 1/16 DU-equivalents, saturating
    uint16_t due, urgent;                // thresholds, same unit; 0 = never
    uint16_t pan_w, pan_h;               // panel size: clipping and partial edge tiles
} epd_ghost_t;

// Clears the debt, sizes the grid (edge tiles may be partial) and sets both
// thresholds to 0 (never). A panel needing more than EPD_GHOST_MAX_TILES
// tiles gets an empty grid: every call is then a no-op and the level stays OK.
void epd_ghost_init(epd_ghost_t *g, uint16_t pan_w, uint16_t pan_h);
// A threshold is reached at debt >= value; 0 disables that level.
void epd_ghost_set_budget(epd_ghost_t *g, uint16_t due, uint16_t urgent);
// Adds weight16 x (driven pixels in the tile / tile pixels) to every tile the
// rect touches, rounded up so any driven pixel costs at least 1. The rect is
// clipped to the panel; an empty or off-panel rect adds nothing. Edge tiles
// use their own (partial) area, so a rect covering one fully adds weight16.
void epd_ghost_add(epd_ghost_t *g, epd_rect_t driven, uint16_t weight16);   // weight in 1/16
void epd_ghost_clean(epd_ghost_t *g);          // full clean
void epd_ghost_halve(epd_ghost_t *g);          // whole-panel GL16 / wipe
// Worst tile against the thresholds (URGENT wins over DUE).
epd_ghost_level_t epd_ghost_level(const epd_ghost_t *g);
// Debt of the worst tile.
uint16_t epd_ghost_max(const epd_ghost_t *g);

#ifdef __cplusplus
}
#endif
