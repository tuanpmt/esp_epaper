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

epd_run_result_t epd_par_run(const epd_par_run_ops_t *o, uint16_t phases, uint32_t phase_us,
                             uint32_t budget_ms, uint8_t *const buf[2], epd_par_run_stats_t *st)
{
    memset(st, 0, sizeof(*st));
    const uint64_t hv_t0 = o->now_us(o->ctx);
    if (o->hv_on(o->ctx) != ESP_OK) {
        return finish(o, EPD_RUN_HV_FAIL, hv_t0, st);
    }
    const uint64_t budget_us = (uint64_t)budget_ms * 1000u;
    uint64_t c0 = o->now_us(o->ctx);
    if (phases) {
        o->compute(o->ctx, 0, buf[0]);
    }
    st->compute_max_us = (uint32_t)(o->now_us(o->ctx) - c0);
    uint64_t slot = o->now_us(o->ctx);
    for (uint16_t k = 0; k < phases; k++) {
        o->wait_until_us(o->ctx, slot);         // never faster than the phase rate
        const uint64_t start = o->now_us(o->ctx);
        if (start - hv_t0 > budget_us) {
            return finish(o, EPD_RUN_BUDGET, hv_t0, st);
        }
        if (!o->pg_ok(o->ctx)) {
            return finish(o, EPD_RUN_PG_DROP, hv_t0, st);
        }
        if (k && start > slot + phase_us / 4) {
            st->late_phases++;
        }
        slot = start + phase_us;
        if (o->scan_start(o->ctx, buf[k & 1]) != ESP_OK) {
            return finish(o, EPD_RUN_SCAN_FAIL, hv_t0, st);
        }
        if (k + 1 < phases) {
            c0 = o->now_us(o->ctx);
            o->compute(o->ctx, k + 1, buf[(k + 1) & 1]);
            const uint32_t c = (uint32_t)(o->now_us(o->ctx) - c0);
            st->compute_max_us = c > st->compute_max_us ? c : st->compute_max_us;
        }
        if (o->scan_wait(o->ctx) != ESP_OK) {
            return finish(o, EPD_RUN_SCAN_FAIL, hv_t0, st);
        }
        st->phases_done = (uint16_t)(k + 1);
    }
    return finish(o, EPD_RUN_OK, hv_t0, st);
}
