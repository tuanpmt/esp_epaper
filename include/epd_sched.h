#pragma once

// Refresh scheduler: an event-driven state machine with no I/O. It merges
// LVGL frames (dirty panel rects) and refresh intents (mode, seq) into one
// pending run, keeps one run in flight, and reports what the EPD task must do
// next (start a refresh, report a commit, re-init after a BUSY timeout).
// Panel-independent: size, alignment and timeout are parameters.

#include <stdbool.h>
#include <stdint.h>
#include "epd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_SCHED_MAX_RECTS 8

typedef enum { EPD_ACT_NONE, EPD_ACT_START, EPD_ACT_COMMIT, EPD_ACT_REINIT } epd_act_kind_t;

typedef struct {
    epd_act_kind_t kind;
    epd_mode_t mode;                              // START
    epd_rect_t rects[EPD_SCHED_MAX_RECTS];        // START: windows to write (full screen for GRAY/FULL_CLEAN)
    uint8_t n;
    uint32_t seq;                                 // COMMIT: highest intent seq shown (0 = frame without an intent)
    bool error;                                   // COMMIT after a timeout
} epd_act_t;

// Private: use the functions below only
typedef struct {
    uint16_t pan_w, pan_h;
    uint8_t align;
    uint32_t busy_timeout_ms;
    // pending (next run)
    epd_rect_t rects[EPD_SCHED_MAX_RECTS];
    uint8_t n;
    bool has_intent;
    bool awaiting_frame;        // an intent arrived; its LVGL frame has not been rendered yet
    epd_mode_t mode;
    uint32_t seq;
    bool force_full;
    // in flight
    bool busy;
    uint32_t start_ms;
    uint32_t run_seq;
    // queued actions
    bool reinit_due;
    bool commit_due;
    uint32_t commit_seq;
    bool commit_error;
    uint32_t last_seq;                            // highest non-zero seq committed so far
} epd_sched_t;

void epd_sched_init(epd_sched_t *s, uint16_t pan_w, uint16_t pan_h, uint8_t align, uint32_t busy_timeout_ms);
void epd_sched_frame(epd_sched_t *s, const epd_rect_t *panel_rects, uint8_t n);   // LVGL flushed these areas
void epd_sched_request(epd_sched_t *s, epd_mode_t mode, uint32_t seq);            // intent from os_display
// The LVGL render that follows an intent is complete (with or without dirty
// areas): pending intents may start now. Until then a run carrying an intent
// never starts, so an intent never goes out with an older image.
void epd_sched_frame_end(epd_sched_t *s);
void epd_sched_force_full(epd_sched_t *s);       // invert changed / unpark / first frame
void epd_sched_done(epd_sched_t *s, uint32_t now_ms);                              // panel finished
bool epd_sched_next(epd_sched_t *s, uint32_t now_ms, epd_act_t *out);             // one action, false = idle
uint32_t epd_sched_wait_ms(const epd_sched_t *s, uint32_t now_ms);                // until timeout check, UINT32_MAX idle
bool epd_sched_idle(const epd_sched_t *s);       // nothing pending, nothing in flight (park may proceed)

#ifdef __cplusplus
}
#endif
