#pragma once

// One refresh of a parallel panel: rails on, then for every waveform phase
// check PWR_GOOD, start the scan of phase k, compute phase k+1 into the other
// buffer while the bus is busy, wait for the scan, keep the phase rate (never
// faster than phase_us), and rails off at the end or on any error. Pure
// control flow over injected operations, host-tested; the caller supplies
// the PMIC (epd_tps65185), the scan bus (epd_par_bus) and the compute step
// (epd_wf_pair_lut + epd_wf_phase).

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *ctx;
    esp_err_t (*hv_on)(void *ctx);
    esp_err_t (*hv_off)(void *ctx);
    bool (*pg_ok)(void *ctx);
    // Fills one 2 bpp phase buffer; returns false instead when the phase drives
    // exactly like the previous one (buf untouched: the previous buffer is scanned again)
    bool (*compute)(void *ctx, int phase, uint8_t *buf);
    esp_err_t (*scan_start)(void *ctx, const uint8_t *buf);
    esp_err_t (*scan_wait)(void *ctx);
    uint64_t (*now_us)(void *ctx);
    void (*wait_until_us)(void *ctx, uint64_t t_us);
} epd_par_run_ops_t;

typedef enum {
    EPD_RUN_OK,
    EPD_RUN_HV_FAIL,     // rails did not come up: the panel was not driven
    EPD_RUN_PG_DROP,     // PWR_GOOD lost between phases
    EPD_RUN_SCAN_FAIL,   // bus error or timeout
    EPD_RUN_BUDGET,      // high-voltage time budget exceeded
} epd_run_result_t;

typedef struct {
    uint16_t phases_done;       // phases fully scanned
    bool unknown;               // aborted after driving: the panel image is unknown
    uint32_t hv_us;             // rails on to rails off
    uint32_t compute_max_us;    // slowest phase computation
    uint32_t late_phases;       // phases that started > phase_us / 4 after their slot
    uint16_t reused;            // phases that rescanned the previous buffer
} epd_par_run_stats_t;

epd_run_result_t epd_par_run(const epd_par_run_ops_t *ops, uint16_t phases, uint32_t phase_us,
                             uint32_t budget_ms, uint8_t *const buf[2], epd_par_run_stats_t *st);

#ifdef __cplusplus
}
#endif
