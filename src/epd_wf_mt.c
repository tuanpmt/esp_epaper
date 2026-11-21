// Two-core phase computation (see epd_wf_mt.h).

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "epd_wf_mt.h"

typedef struct {
    const uint8_t *lut_a, *lut_b, *cur, *next;
    const uint16_t *pp;                     // pair plane: replaces cur/next when set
    uint8_t *out_a, *out_b;
    size_t ps, os;
    epd_rect_t band;
} job_t;

static TaskHandle_t s_worker;
static SemaphoreHandle_t s_done, s_mx;
static job_t s_job;

static void run_job(const job_t *j)
{
    if (j->pp && j->lut_b) {
        epd_wf_phase2_pp(j->lut_a, j->lut_b, j->pp, j->ps, j->out_a, j->out_b, j->os, j->band);
    } else if (j->pp) {
        epd_wf_phase_pp(j->lut_a, j->pp, j->ps, j->out_a, j->os, j->band);
    } else if (j->lut_b) {
        epd_wf_phase2(j->lut_a, j->lut_b, j->cur, j->next, j->ps, j->out_a, j->out_b, j->os, j->band);
    } else {
        epd_wf_phase(j->lut_a, j->cur, j->next, j->ps, j->out_a, j->os, j->band);
    }
}

static void worker(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        run_job(&s_job);
        xSemaphoreGive(s_done);
    }
}

esp_err_t epd_wf_mt_init(int worker_core, int priority)
{
    if (s_worker) {
        return ESP_OK;
    }
    s_done = xSemaphoreCreateBinary();
    s_mx = xSemaphoreCreateMutex();
    if (!s_done || !s_mx) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(worker, "epd_wf", 2048, NULL, priority, &s_worker, worker_core) != pdPASS) {
        s_worker = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void split(job_t j)
{
    if (!s_worker || j.band.h < 2 || j.band.w <= 0) {
        run_job(&j);
        return;
    }
    xSemaphoreTake(s_mx, portMAX_DELAY);
    const int16_t top = (int16_t)(j.band.h / 2);
    s_job = j;
    s_job.band.y = (int16_t)(j.band.y + top);
    s_job.band.h = (int16_t)(j.band.h - top);
    xTaskNotifyGive(s_worker);
    j.band.h = top;
    run_job(&j);
    xSemaphoreTake(s_done, portMAX_DELAY);
    xSemaphoreGive(s_mx);
}

void epd_wf_mt_phase(const uint8_t *lut, const uint8_t *cur, const uint8_t *next, size_t plane_stride,
                     uint8_t *out, size_t out_stride, epd_rect_t band)
{
    split((job_t){.lut_a = lut, .cur = cur, .next = next, .out_a = out, .ps = plane_stride, .os = out_stride,
                  .band = band});
}

void epd_wf_mt_phase2(const uint8_t *lut_a, const uint8_t *lut_b, const uint8_t *cur, const uint8_t *next,
                      size_t plane_stride, uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band)
{
    split((job_t){.lut_a = lut_a, .lut_b = lut_b, .cur = cur, .next = next, .out_a = out_a, .out_b = out_b,
                  .ps = plane_stride, .os = out_stride, .band = band});
}

void epd_wf_mt_phase_pp(const uint8_t *lut, const uint16_t *pp, size_t pp_stride, uint8_t *out, size_t out_stride,
                        epd_rect_t band)
{
    split((job_t){.lut_a = lut, .pp = pp, .out_a = out, .ps = pp_stride, .os = out_stride, .band = band});
}

void epd_wf_mt_phase2_pp(const uint8_t *lut_a, const uint8_t *lut_b, const uint16_t *pp, size_t pp_stride,
                         uint8_t *out_a, uint8_t *out_b, size_t out_stride, epd_rect_t band)
{
    split((job_t){.lut_a = lut_a, .lut_b = lut_b, .pp = pp, .out_a = out_a, .out_b = out_b, .ps = pp_stride,
                  .os = out_stride, .band = band});
}
