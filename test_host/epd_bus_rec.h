#pragma once

// Recording fake of epd_bus_t for host tests: every call becomes one entry.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "epd_bus.h"

typedef enum {
    EPD_REC_CMD,    // cmd + its data bytes
    EPD_REC_DATA,   // bulk data
    EPD_REC_RESET,
    EPD_REC_DELAY,  // ms in .ms
    EPD_REC_ARM,    // arm_busy_irq
} epd_rec_type_t;

typedef struct {
    epd_rec_type_t type;
    uint8_t cmd;        // EPD_REC_CMD only
    size_t len;         // number of bytes
    uint8_t *bytes;     // owned copy (NULL when len == 0)
    uint32_t ms;        // EPD_REC_DELAY only
} epd_rec_t;

typedef struct {
    epd_bus_t bus;          // first member: (epd_bus_t *)&rec works
    epd_rec_t *log;
    size_t n, cap;
    uint32_t clock_ms;      // fake now_ms(); advanced by delay_ms
    uint32_t tick_ms;       // every delay_ms sleeps at least this long (0 = exactly ms), like an RTOS tick
    bool busy_level;        // what busy() returns; set by the test
    uint32_t low_reads;     // the next low_reads busy() calls return false (BUSY not risen yet)
    bool reset_seen;        // arm before the first reset is refused, like the device
    epd_busy_cb_t cb;       // armed callback, NULL when not armed
    void *cb_ctx;
} epd_bus_rec_t;

void epd_bus_rec_init(epd_bus_rec_t *r);
void epd_bus_rec_free(epd_bus_rec_t *r);
void epd_bus_rec_clear(epd_bus_rec_t *r);                 // drop the log, keep state
void epd_bus_rec_set_busy(epd_bus_rec_t *r, bool busy);  // does not fire the callback
// Simulate the BUSY interrupt: BUSY goes idle and an armed callback runs once.
// Returns true when a callback ran.
bool epd_bus_rec_fire_busy(epd_bus_rec_t *r);
// Index of the first EPD_REC_CMD with this cmd at or after from, or -1
int epd_bus_rec_find_cmd(const epd_bus_rec_t *r, uint8_t cmd, size_t from);
