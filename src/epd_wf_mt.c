// Two-core waveform work (see epd_wf_mt.h). One worker per core: the bottom
// half of the rows always goes to the worker of the core the caller is NOT
// on, so the halves really run side by side whatever core the caller uses
// (an unpinned console task on the worker's core serialised them before).

#include "sdkconfig.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "epd_wf_mt.h"

typedef enum { J_PHASE, J_PHASE2, J_PHASE_PP, J_PHASE2_PP, J_PHASE_TP, J_PAIR_PLANE, J_PAIR_PLANE_BIN, J_TRANS_PLANE } kind_t;

typedef struct {
    kind_t kind;
    const uint8_t *lut_a, *lut_b, *cur, *next;
    const uint8_t (*tabs)[256];
    const uint16_t *pp_in;
    const uint8_t *tp_in;
    uint16_t *pp_out;
    uint8_t *tp_out, *out_a, *out_b;
    size_t ps, os;
    epd_rect_t band;
} job_t;

#define CORES 2

static TaskHandle_t s_worker[CORES];
static job_t s_job[CORES];
static SemaphoreHandle_t s_done, s_mx;
static uint32_t s_own_max, s_wait_max;     // caller half / waiting for the other half, us

static void run_job(const job_t *j)
{
    switch (j->kind) {
    case J_PHASE:
        epd_wf_phase(j->lut_a, j->cur, j->next, j->ps, j->out_a, j->os, j->band);
        break;
    case J_PHASE2:
        epd_wf_phase2(j->lut_a, j->lut_b, j->cur, j->next, j->ps, j->out_a, j->out_b, j->os, j->band);
        break;
    case J_PHASE_PP:
        epd_wf_phase_pp(j->lut_a, j->pp_in, j->ps, j->out_a, j->os, j->band);
        break;
    case J_PHASE2_PP:
        epd_wf_phase2_pp(j->lut_a, j->lut_b, j->pp_in, j->ps, j->out_a, j->out_b, j->os, j->band);
        break;
    case J_PHASE_TP:
        epd_wf_phase_tp(j->tabs, j->tp_in, j->ps, j->out_a, j->os, j->band);
        break;
    case J_PAIR_PLANE:
        epd_wf_pair_plane(j->cur, j->next, j->ps, j->pp_out, j->band);
        break;
    case J_PAIR_PLANE_BIN:
        epd_wf_pair_plane_bin(j->cur, j->next, j->ps, j->pp_out, j->band);
        break;
    case J_TRANS_PLANE:
        epd_wf_trans_plane(j->cur, j->next, j->ps, j->tp_out, j->band);
        break;
    }
}

static void worker(void *arg)
{
    const int core = (int)(intptr_t)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        run_job(&s_job[core]);
        xSemaphoreGive(s_done);
    }
}

esp_err_t epd_wf_mt_init(int priority)
{
    if (s_mx) {
        return ESP_OK;
    }
    s_done = xSemaphoreCreateBinary();
    SemaphoreHandle_t mx = xSemaphoreCreateMutex();
    if (!s_done || !mx) {
        return ESP_ERR_NO_MEM;
    }
    for (int c = 0; c < CORES; c++) {
#if CONFIG_SPIRAM && CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
        // Stack in PSRAM (save internal RAM): the workers only compute, no flash access
        const UBaseType_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
#else
        const UBaseType_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
#endif
        if (xTaskCreatePinnedToCoreWithCaps(worker, c ? "epd_wf1" : "epd_wf0", 2048, (void *)(intptr_t)c, priority,
                                            &s_worker[c], c, caps) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    s_mx = mx;
    return ESP_OK;
}

static void split(job_t j)
{
    if (!s_mx || j.band.h < 2 || j.band.w <= 0) {
        run_job(&j);
        return;
    }
    xSemaphoreTake(s_mx, portMAX_DELAY);
    // An unpinned caller that migrates right after this read only makes the
    // halves run one after the other: slower, still correct
    const int other = xPortGetCoreID() ^ 1;
    const int16_t top = (int16_t)(j.band.h / 2);
    s_job[other] = j;
    s_job[other].band.y = (int16_t)(j.band.y + top);
    s_job[other].band.h = (int16_t)(j.band.h - top);
    xTaskNotifyGive(s_worker[other]);
    j.band.h = top;
    const int64_t t0 = esp_timer_get_time();
    run_job(&j);
    const int64_t t1 = esp_timer_get_time();
    xSemaphoreTake(s_done, portMAX_DELAY);
    const uint32_t own = (uint32_t)(t1 - t0), wait = (uint32_t)(esp_timer_get_time() - t1);
    s_own_max = own > s_own_max ? own : s_own_max;
    s_wait_max = wait > s_wait_max ? wait : s_wait_max;
    xSemaphoreGive(s_mx);
}

void epd_wf_mt_phase(const uint8_t *lut, const uint8_t *cur, const uint8_t *next, size_t plane_stride,
                     uint8_t *out, size_t out_stride, epd_rect_t band)
{
    split((job_t){.kind = J_PHASE, .lut_a = lut, .cur = cur, .next = next, .out_a = out, .ps = plane_stride,
                  .os = out_stride, .band = band});
}

void epd_wf_mt_phase2(const uint8_t *lut_a, const uint8_t *lut_b, const uint8_t *cur, const uint8_t *next,
                      size_t plane_stride, uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band)
{
    split((job_t){.kind = J_PHASE2, .lut_a = lut_a, .lut_b = lut_b, .cur = cur, .next = next, .out_a = out_a,
                  .out_b = out_b, .ps = plane_stride, .os = out_stride, .band = band});
}

void epd_wf_mt_phase_pp(const uint8_t *lut, const uint16_t *pp, size_t pp_stride, uint8_t *out, size_t out_stride,
                        epd_rect_t band)
{
    split((job_t){.kind = J_PHASE_PP, .lut_a = lut, .pp_in = pp, .out_a = out, .ps = pp_stride, .os = out_stride,
                  .band = band});
}

void epd_wf_mt_phase2_pp(const uint8_t *lut_a, const uint8_t *lut_b, const uint16_t *pp, size_t pp_stride,
                         uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band)
{
    split((job_t){.kind = J_PHASE2_PP, .lut_a = lut_a, .lut_b = lut_b, .pp_in = pp, .out_a = out_a, .out_b = out_b,
                  .ps = pp_stride, .os = out_stride, .band = band});
}

void epd_wf_mt_phase_tp(const uint8_t t[4][256], const uint8_t *tp, size_t tp_stride, uint8_t *out,
                        size_t out_stride, epd_rect_t band)
{
    split((job_t){.kind = J_PHASE_TP, .tabs = t, .tp_in = tp, .out_a = out, .ps = tp_stride, .os = out_stride,
                  .band = band});
}

void epd_wf_mt_pair_plane(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint16_t *pp,
                          epd_rect_t band)
{
    split((job_t){.kind = J_PAIR_PLANE, .cur = cur, .next = next, .pp_out = pp, .ps = plane_stride, .band = band});
}

void epd_wf_mt_pair_plane_bin(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint16_t *pp,
                              epd_rect_t band)
{
    split((job_t){.kind = J_PAIR_PLANE_BIN, .cur = cur, .next = next, .pp_out = pp, .ps = plane_stride,
                  .band = band});
}

void epd_wf_mt_trans_plane(const uint8_t *cur, const uint8_t *next, size_t plane_stride, uint8_t *tp,
                           epd_rect_t band)
{
    split((job_t){.kind = J_TRANS_PLANE, .cur = cur, .next = next, .tp_out = tp, .ps = plane_stride, .band = band});
}

void epd_wf_mt_set_priority(int priority)
{
    for (int c = 0; c < CORES; c++) {
        if (s_worker[c]) {
            vTaskPrioritySet(s_worker[c], (UBaseType_t)priority);
        }
    }
}

void epd_wf_mt_take_stats(uint32_t *own_max_us, uint32_t *wait_max_us)
{
    *own_max_us = s_own_max;
    *wait_max_us = s_wait_max;
    s_own_max = s_wait_max = 0;
}
