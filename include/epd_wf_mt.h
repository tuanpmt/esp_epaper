#pragma once

// Waveform work split over two cores (FreeRTOS, device only). The caller
// computes the top half of the band's rows, a worker pinned to the other core
// (one worker per core) the bottom half; the call returns when both are done.
// Without workers (init not called or failed) it runs on the caller alone.

#include <stdint.h>
#include "esp_err.h"
#include "epd_wf.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t epd_wf_mt_init(int priority);    // idempotent; one worker per core
void epd_wf_mt_phase(const uint8_t *lut, const uint8_t *cur, const uint8_t *next, size_t plane_stride,
                     uint8_t *out, size_t out_stride, epd_rect_t band);
void epd_wf_mt_phase2(const uint8_t *lut_a, const uint8_t *lut_b, const uint8_t *cur, const uint8_t *next,
                      size_t plane_stride, uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band);

void epd_wf_mt_phase_pp(const uint8_t *lut, const uint16_t *pp, size_t pp_stride, uint8_t *out, size_t out_stride,
                        epd_rect_t band);
void epd_wf_mt_phase2_pp(const uint8_t *lut_a, const uint8_t *lut_b, const uint16_t *pp, size_t pp_stride,
                         uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band);
void epd_wf_mt_phase_tp(const uint8_t t[4][256], const uint8_t *tp, size_t tp_stride, uint8_t *out,
                        size_t out_stride, epd_rect_t band);
void epd_wf_mt_pair_plane(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint16_t *pp,
                          epd_rect_t band);
// DU kinds: binary targets (epd_wf_pair_plane_bin)
void epd_wf_mt_pair_plane_bin(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint16_t *pp,
                              epd_rect_t band);
void epd_wf_mt_trans_plane(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint8_t *tp,
                           epd_rect_t band);
// Both workers' priority (e.g. raised for the duration of a refresh)
void epd_wf_mt_set_priority(int priority);
// Since the last call: slowest caller half, and longest wait for the other
// core's half after the caller's was done (high = the other core was busy)
void epd_wf_mt_take_stats(uint32_t *own_max_us, uint32_t *wait_max_us);

#ifdef __cplusplus
}
#endif
