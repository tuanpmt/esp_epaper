// v3 SPI bus for e-paper controllers: DMA transfers through an internal bounce
// buffer, DC switched from the task between command and data transactions, and
// a one-shot BUSY interrupt.
//
// Lessons from the stock 3.97" firmware (CloudZao/Metalio-E-INK4,
// main/boards/metalio-e-ink-4/display/esp_lcd_panel_ssd1677.c, MIT):
// - no DMA straight from PSRAM: data is copied into internal DMA RAM first;
// - RST is push-pull and idles high;
// - BUSY has a pull-down; its interrupt stays disabled until it is armed, and it
//   is never armed before the first reset (enabling it earlier caused ISR storms
//   and an interrupt watchdog reset).

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "epd_bus.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "epd_bus";

#define BOUNCE_DEFAULT 4096
#define QUEUE_SIZE 4

typedef struct {
    epd_bus_t base;                 // first member
    epd_bus_spi_cfg_t cfg;
    spi_device_handle_t dev;
    uint8_t *bounce;                // cfg.bounce bytes, internal DMA RAM
    size_t half;                    // ping-pong chunk size (two halves of bounce)
    bool reset_done;                // BUSY interrupt may only be armed after a reset
    bool isr_added;
    portMUX_TYPE lock;              // guards cb/cb_ctx between task and ISR
    epd_busy_cb_t cb;               // armed callback, NULL when disarmed
    void *cb_ctx;
} epd_bus_spi_t;

static inline bool level_busy(const epd_bus_spi_t *s)
{
    return gpio_get_level(s->cfg.busy) == (s->cfg.busy_active_high ? 1 : 0);
}

// Take the armed callback (at most one taker wins); ISR and task safe
static IRAM_ATTR epd_busy_cb_t claim_cb(epd_bus_spi_t *s, void **ctx)
{
    portENTER_CRITICAL_SAFE(&s->lock);
    epd_busy_cb_t cb = s->cb;
    *ctx = s->cb_ctx;
    s->cb = NULL;
    portEXIT_CRITICAL_SAFE(&s->lock);
    return cb;
}

// The GPIO ISR service is installed without ESP_INTR_FLAG_IRAM, so this handler
// only runs with the flash cache enabled and may call the (flash) GPIO driver.
static void IRAM_ATTR busy_isr(void *arg)
{
    epd_bus_spi_t *s = arg;
    if (level_busy(s)) {
        return;     // stale edge latched while disarmed: stay armed
    }
    gpio_intr_disable(s->cfg.busy);
    void *ctx;
    epd_busy_cb_t cb = claim_cb(s, &ctx);
    if (cb) {
        cb(ctx);
    }
}

// Queue len bytes (DC already high) through the two bounce halves; CS stays
// active between chunks. Caller holds the bus.
static esp_err_t send_bulk(epd_bus_spi_t *s, const uint8_t *data, size_t len)
{
    spi_transaction_t tr[2];
    spi_transaction_t *done;
    int inflight = 0, k = 0;
    esp_err_t err = ESP_OK;
    size_t off = 0;
    while (off < len) {
        if (inflight == 2) {    // the oldest transaction used bounce half k
            err = spi_device_get_trans_result(s->dev, &done, portMAX_DELAY);
            if (err != ESP_OK) {
                break;
            }
            inflight--;
        }
        size_t n = len - off < s->half ? len - off : s->half;
        uint8_t *buf = s->bounce + (size_t)k * s->half;
        memcpy(buf, data + off, n);
        memset(&tr[k], 0, sizeof tr[k]);
        tr[k].length = n * 8;
        tr[k].tx_buffer = buf;
        tr[k].flags = off + n < len ? SPI_TRANS_CS_KEEP_ACTIVE : 0;
        err = spi_device_queue_trans(s->dev, &tr[k], portMAX_DELAY);
        if (err != ESP_OK) {
            break;
        }
        inflight++;
        off += n;
        k ^= 1;
    }
    while (inflight > 0) {
        esp_err_t e = spi_device_get_trans_result(s->dev, &done, portMAX_DELAY);
        if (err == ESP_OK) {
            err = e;
        }
        inflight--;
    }
    return err;
}

static esp_err_t bus_cmd(epd_bus_t *b, uint8_t cmd, const uint8_t *data, size_t len)
{
    epd_bus_spi_t *s = (epd_bus_spi_t *)b;
    if (len && !data) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = spi_device_acquire_bus(s->dev, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(s->cfg.dc, 0);
    spi_transaction_t t = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA | (len ? SPI_TRANS_CS_KEEP_ACTIVE : 0),
        .tx_data = {cmd},
    };
    err = spi_device_polling_transmit(s->dev, &t);
    gpio_set_level(s->cfg.dc, 1);
    if (err == ESP_OK && len) {
        err = send_bulk(s, data, len);
    }
    spi_device_release_bus(s->dev);
    return err;
}

static esp_err_t bus_data(epd_bus_t *b, const uint8_t *data, size_t len)
{
    epd_bus_spi_t *s = (epd_bus_spi_t *)b;
    if (len == 0) {
        return ESP_OK;
    }
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = spi_device_acquire_bus(s->dev, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(s->cfg.dc, 1);
    err = send_bulk(s, data, len);
    spi_device_release_bus(s->dev);
    return err;
}

static bool bus_busy(epd_bus_t *b)
{
    return level_busy((epd_bus_spi_t *)b);
}

static esp_err_t busy_isr_setup(epd_bus_spi_t *s)
{
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   // INVALID_STATE: already installed
        return err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(s->cfg.busy, busy_isr, s), TAG, "BUSY isr add");
    gpio_intr_disable(s->cfg.busy);   // handler_add may enable it
    // Edge into the idle level: falling for an active-high BUSY
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(s->cfg.busy,
                                           s->cfg.busy_active_high ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE),
                        TAG, "BUSY intr type");
    gpio_intr_disable(s->cfg.busy);
    s->isr_added = true;
    return ESP_OK;
}

// One-shot. If BUSY is already idle (refresh done before arming, or the edge
// raced the arming), cb runs here in the caller's context instead of the ISR.
static esp_err_t bus_arm_busy_irq(epd_bus_t *b, epd_busy_cb_t cb, void *ctx)
{
    epd_bus_spi_t *s = (epd_bus_spi_t *)b;
    if (!cb) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s->reset_done) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s->isr_added) {
        ESP_RETURN_ON_ERROR(busy_isr_setup(s), TAG, "BUSY isr setup");
    }
    portENTER_CRITICAL(&s->lock);
    s->cb = cb;
    s->cb_ctx = ctx;
    portEXIT_CRITICAL(&s->lock);
    gpio_intr_enable(s->cfg.busy);
    if (!level_busy(s)) {
        gpio_intr_disable(s->cfg.busy);
        void *c;
        epd_busy_cb_t f = claim_cb(s, &c);
        if (f) {    // NULL when the ISR got there first
            f(c);
        }
    }
    return ESP_OK;
}

static void bus_delay_ms(epd_bus_t *b, uint32_t ms)
{
    (void)b;
    if (ms) {
        TickType_t t = pdMS_TO_TICKS(ms);
        vTaskDelay(t ? t : 1);
    }
}

// Disarms a pending BUSY callback: the panel state it waited for is gone.
static void bus_reset(epd_bus_t *b)
{
    epd_bus_spi_t *s = (epd_bus_spi_t *)b;
    if (s->isr_added) {
        gpio_intr_disable(s->cfg.busy);
    }
    portENTER_CRITICAL(&s->lock);
    s->cb = NULL;
    portEXIT_CRITICAL(&s->lock);
    gpio_set_level(s->cfg.rst, 0);
    bus_delay_ms(b, 10);
    gpio_set_level(s->cfg.rst, 1);
    bus_delay_ms(b, 10);
    s->reset_done = true;
}

esp_err_t epd_bus_spi_new(const epd_bus_spi_cfg_t *cfg, epd_bus_t **out)
{
    ESP_RETURN_ON_FALSE(cfg && out && cfg->hz, ESP_ERR_INVALID_ARG, TAG, "invalid arg");
    esp_err_t ret = ESP_OK;
    bool bus_owned = false;
    epd_bus_spi_t *s = calloc(1, sizeof *s);
    ESP_RETURN_ON_FALSE(s, ESP_ERR_NO_MEM, TAG, "no mem");
    s->cfg = *cfg;
    if (!s->cfg.bounce) {
        s->cfg.bounce = BOUNCE_DEFAULT;
    }
    s->half = (s->cfg.bounce / 2) & ~(size_t)3;   // word-aligned halves for DMA
    ESP_GOTO_ON_FALSE(s->half > 0, ESP_ERR_INVALID_ARG, fail, TAG, "bounce too small");
    portMUX_INITIALIZE(&s->lock);
    s->bounce = heap_caps_malloc(2 * s->half, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ESP_GOTO_ON_FALSE(s->bounce, ESP_ERR_NO_MEM, fail, TAG, "bounce alloc");

    // DC and RST: push-pull outputs, idle high (RST released)
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << cfg->dc) | (1ULL << cfg->rst),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&out_cfg), fail, TAG, "DC/RST gpio");
    gpio_set_level(cfg->dc, 1);
    gpio_set_level(cfg->rst, 1);

    // BUSY: input with pull-down, interrupt disabled until armed after a reset
    gpio_config_t busy_cfg = {
        .pin_bit_mask = 1ULL << cfg->busy,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&busy_cfg), fail, TAG, "BUSY gpio");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->mosi,
        .miso_io_num = -1,
        .sclk_io_num = cfg->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (int)s->half,
    };
    ret = spi_bus_initialize((spi_host_device_t)cfg->host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_ERR_INVALID_STATE) {
        // Already initialized by someone else: share it (its max_transfer_sz must be >= half)
        ESP_LOGI(TAG, "SPI host %d already initialized, sharing it", cfg->host);
        ret = ESP_OK;
    } else if (ret == ESP_OK) {
        bus_owned = true;
    }
    ESP_GOTO_ON_ERROR(ret, fail, TAG, "SPI bus init");

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = (int)cfg->hz,
        .mode = 0,
        .spics_io_num = cfg->cs,
        .queue_size = QUEUE_SIZE,
    };
    ESP_GOTO_ON_ERROR(spi_bus_add_device((spi_host_device_t)cfg->host, &dev_cfg, &s->dev), fail, TAG, "SPI add device");

    s->base.cmd = bus_cmd;
    s->base.data = bus_data;
    s->base.busy = bus_busy;
    s->base.arm_busy_irq = bus_arm_busy_irq;
    s->base.reset = bus_reset;
    s->base.delay_ms = bus_delay_ms;
    *out = &s->base;
    ESP_LOGI(TAG, "SPI%d %lu Hz, bounce 2x%u", cfg->host + 1, (unsigned long)cfg->hz, (unsigned)s->half);
    return ESP_OK;

fail:
    if (bus_owned) {
        spi_bus_free((spi_host_device_t)cfg->host);
    }
    heap_caps_free(s->bounce);
    free(s);
    return ret;
}
