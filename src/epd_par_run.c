// Refresh run loop for parallel panels (see epd_par_run.h).

#include <string.h>
#include "epd_par_run.h"

static epd_run_result_t finish(const epd_par_run_ops_t *o, epd_run_result_t r, uint64_t hv_t0,
                               epd_par_run_stats_t *st)
{
    o->hv_off(o->ctx);      // every path: rails off exactly once
    st->hv_us = (uint32_t)(o->now_us(o->ctx) - hv_t0);
    st->unknown = r != EPD_RUN_OK && r != EPD_RUN_HV_FAIL;
    return r;
}

epd_run_result_t epd_par_run(const epd_par_run_ops_t *o, uint16_t phases, uint32_t phase_hz,
                             uint32_t budget_ms, uint8_t *const buf[2], epd_par_run_stats_t *st)
{
    memset(st, 0, sizeof(*st));
    const uint64_t hv_t0 = o->now_us(o->ctx);
    if (o->hv_on(o->ctx) != ESP_OK) {
        return finish(o, EPD_RUN_HV_FAIL, hv_t0, st);
    }
    // The budget limits the time with the rails up, from the return of hv_on:
    // hv_on's own time (PMIC wake from SLEEP, register writes, PWR_GOOD wait)
    // varies with the I2C traffic and must not eat the scan time (device
    // stage d: GC16 aborted after 14 phases when hv_on took ~0.85 s). The
    // BSP watchdog still bounds PWRUP high independently.
    const uint64_t rails_t0 = o->now_us(o->ctx);
    st->hv_on_us = (uint32_t)(rails_t0 - hv_t0);
    const uint64_t budget_us = (uint64_t)budget_ms * 1000u;
    uint64_t c0 = o->now_us(o->ctx);
    if (phases) {
        o->compute(o->ctx, 0, buf[0]);
    }
    st->compute_max_us = (uint32_t)(o->now_us(o->ctx) - c0);
    const uint32_t phase_us = 1000000u / phase_hz;
    uint64_t origin = o->now_us(o->ctx);
    int bi = 0;     // buffer holding phase k
    for (uint16_t k = 0; k < phases; k++) {
        // one timeline from phase 0: never faster than the phase rate, and a
        // late phase does not move the later targets
        const uint64_t slot = origin + ((uint64_t)k * 1000000u + phase_hz - 1u) / phase_hz;
        o->wait_until_us(o->ctx, slot);
        const uint64_t start = o->now_us(o->ctx);
        if (k == 0) {
            origin = start;
        }
        // Budget: checked once, before phase 0 is scanned. Cutting a waveform
        // in the middle leaves the panel in its dark phases (device stage d:
        // GC16 cut after 15 phases showed a black page with a white UI), so
        // after phase 0 only real faults stop the run: scan errors, PWR_GOOD
        // drop, and the BSP high-voltage watchdog outside this loop.
        if (k == 0 && start - rails_t0 + (uint64_t)phases * phase_us > budget_us) {
            const epd_run_result_t r = finish(o, EPD_RUN_BUDGET, hv_t0, st);
            st->unknown = false;    // nothing was scanned: the panel keeps its image
            return r;
        }
        if (k && start > slot + phase_us / 4) {
            st->late_phases++;
        }
        const uint32_t lag = (uint32_t)(start - slot);
        st->lag_us_max = lag > st->lag_us_max ? lag : st->lag_us_max;
        if (o->scan_start(o->ctx, buf[bi]) != ESP_OK) {
            return finish(o, EPD_RUN_SCAN_FAIL, hv_t0, st);
        }
        // While the frame scans: PWR_GOOD (an I2C read is longer than the
        // slack between two frames of the timeline), then the next phase
        uint64_t t = o->now_us(o->ctx);
        const bool pg = o->pg_ok(o->ctx);
        const uint32_t pg_us = (uint32_t)(o->now_us(o->ctx) - t);
        st->pg_us_max = pg_us > st->pg_us_max ? pg_us : st->pg_us_max;
        if (!pg) {
            const epd_run_result_t r = finish(o, EPD_RUN_PG_DROP, hv_t0, st);    // XOE low, rails off now
            o->scan_wait(o->ctx);       // then let the undriven frame end
            return r;
        }
        int next_bi = bi;
        if (k + 1 < phases) {
            c0 = o->now_us(o->ctx);
            if (o->compute(o->ctx, k + 1, buf[bi ^ 1])) {
                next_bi = bi ^ 1;   // never the buffer the bus is reading
            } else {
                st->reused++;
            }
            const uint32_t c = (uint32_t)(o->now_us(o->ctx) - c0);
            st->compute_max_us = c > st->compute_max_us ? c : st->compute_max_us;
        }
        if (o->scan_wait(o->ctx) != ESP_OK) {
            return finish(o, EPD_RUN_SCAN_FAIL, hv_t0, st);
        }
        const uint32_t fr = (uint32_t)(o->now_us(o->ctx) - start);
        st->frame_us_max = fr > st->frame_us_max ? fr : st->frame_us_max;
        st->phases_done = (uint16_t)(k + 1);
        bi = next_bi;
    }
    return finish(o, EPD_RUN_OK, hv_t0, st);
}
