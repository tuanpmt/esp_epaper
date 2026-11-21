#pragma once

// v3 panel bus: the byte transport a controller (SSD1677, ...) talks through.
// The device implementation is SPI with DMA (epd_bus_spi_new); host tests use a
// recording fake (test_host/epd_bus_rec.h) behind the same function pointers.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct epd_bus epd_bus_t;

// BUSY went idle. Called once per arm_busy_irq, from the GPIO ISR (or from the
// arming task when BUSY was already idle). Must be ISR-safe.
typedef void (*epd_busy_cb_t)(void *ctx);

struct epd_bus {
    // One command: DC low command byte, then len data bytes with DC high (len may be 0)
    esp_err_t (*cmd)(epd_bus_t *b, uint8_t cmd, const uint8_t *data, size_t len);
    // Bulk data (DC high) following a command; copied through an internal bounce buffer,
    // so data may live in PSRAM or flash
    esp_err_t (*data)(epd_bus_t *b, const uint8_t *data, size_t len);
    // true while the panel is busy
    bool (*busy)(epd_bus_t *b);
    // One-shot: cb runs when BUSY goes idle. Arm just before activation, never before
    // the first reset (ESP_ERR_INVALID_STATE). If BUSY is already idle, cb runs at once
    // from the caller's context.
    esp_err_t (*arm_busy_irq)(epd_bus_t *b, epd_busy_cb_t cb, void *ctx);
    // RST 10 ms low, 10 ms high (does not wait for BUSY)
    void (*reset)(epd_bus_t *b);
    void (*delay_ms)(epd_bus_t *b, uint32_t ms);
};

typedef struct {
    int host;               // spi_host_device_t, e.g. SPI3_HOST
    int mosi, sclk, cs, dc, rst, busy;   // GPIO numbers
    uint32_t hz;            // SPI clock
    bool busy_active_high;  // BUSY level while the panel is busy
    size_t bounce;          // internal DMA bounce buffer bytes (0 = 4096)
} epd_bus_spi_cfg_t;

// SPI with DMA and queued transactions. The SPI bus is initialized here unless the
// host already is (then the existing bus is shared). Device only.
esp_err_t epd_bus_spi_new(const epd_bus_spi_cfg_t *cfg, epd_bus_t **out);

#ifdef __cplusplus
}
#endif
