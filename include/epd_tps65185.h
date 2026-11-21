#pragma once

// TPS65185 e-paper PMIC sequencer (high-voltage rails for parallel panels).
//
// A pure state machine over injected operations: I2C register access, the
// three control lines (WAKEUP, PWRUP, VCOM_CTRL, on an IO expander or GPIOs),
// the PWR_GOOD pin, the panel output-enable lines and a delay. No ESP-IDF
// driver dependency, so the sequence is host-tested byte for byte.
//
// Power-up order (stricter than the vendor firmware and EPDiy):
//   WAKEUP=1, 50 ms, probe, REVID, read INT1/INT2 (clears them)
//   UPSEQ0/1 and VCOM1/VCOM2 written and read back while in STANDBY
//   PWRUP=1, wait for the PWR_GOOD pin, ENABLE=0x3F, wait for PG register
//   bits 0xFA, then VCOM_CTRL=1 last
// Power-down: output enable low, 12 us, VCOM_CTRL=0, PWRUP=0, wait PG low.
//
// Refusals: no I2C while WAKEUP=0 (the chip NACKs); VCOM2 writes with any
// bit but bit 0 (bit 6 PROG burns VCOM into the PMIC's NVM); ENABLE is never
// written from outside the sequencer; VCOM changes only with rails off.
// Two failed power-ups in a row latch EPD_TPS_FAILED until reboot.
//
// Register map and sequencing values follow the vendor driver of the Metalio
// E-Ink 4 Plus (CloudZao Metalio-E-INK4-Plus, bus/tps65185.c, MIT).

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_TPS_ADDR            0x68
#define EPD_TPS_REG_TMST_VALUE  0x00
#define EPD_TPS_REG_ENABLE      0x01
#define EPD_TPS_REG_VADJ        0x02
#define EPD_TPS_REG_VCOM1       0x03
#define EPD_TPS_REG_VCOM2       0x04
#define EPD_TPS_REG_INT_EN1     0x05
#define EPD_TPS_REG_INT_EN2     0x06
#define EPD_TPS_REG_INT1        0x07
#define EPD_TPS_REG_INT2        0x08
#define EPD_TPS_REG_UPSEQ0      0x09
#define EPD_TPS_REG_UPSEQ1      0x0A
#define EPD_TPS_REG_DWNSEQ0     0x0B
#define EPD_TPS_REG_DWNSEQ1     0x0C
#define EPD_TPS_REG_TMST1       0x0D
#define EPD_TPS_REG_TMST2       0x0E
#define EPD_TPS_REG_PG          0x0F
#define EPD_TPS_REG_REVID       0x10

#define EPD_TPS_REVID           0x65    // measured on our unit (verification-eink4-plus.md)
#define EPD_TPS_UPSEQ0          0xE4    // datasheet defaults, as the vendor firmware writes them
#define EPD_TPS_UPSEQ1          0x55
#define EPD_TPS_ENABLE_RAILS    0x3F    // as vendor firmware and EPDiy, after PWR_GOOD only
#define EPD_TPS_PG_MASK         0xFA    // PG register bits that must all be set
#define EPD_TPS_VCOM2_ALLOWED   0x01    // VCOM2: only bit 0 (VCOM bit 8) may ever be written
#define EPD_TPS_INT1_THERMAL    0x60    // INT1 TSD | HOT: refuse to power up (verify in stage A0)
#define EPD_TPS_TMST1_READ      0x80    // TMST1: start a thermistor conversion
#define EPD_TPS_TMST1_DONE      0x20    // TMST1: conversion done

#define EPD_TPS_VCOM_MIN_MV     1000
#define EPD_TPS_VCOM_MAX_MV     3000
#define EPD_TPS_VCOM_DEFAULT_MV 1200

#define EPD_TPS_WAKE_MS         50      // SLEEP -> STANDBY before I2C
#define EPD_TPS_PG_TIMEOUT_MS   100     // PWR_GOOD pin after PWRUP
#define EPD_TPS_PGREG_TIMEOUT_MS 50     // PG register after ENABLE
#define EPD_TPS_OFF_PG_MS       50      // PWR_GOOD low after PWRUP=0
#define EPD_TPS_OE_LEAD_US      12      // XOE low before the rails go (E0470A14-AF 6.1)
#define EPD_TPS_SLEEP_MS        120     // after WAKEUP=0 (datasheet 8.4: >= 100 ms)
#define EPD_TPS_RETRY_MS        100     // between the two power-up attempts

typedef enum { EPD_TPS_LINE_VCOM_CTRL, EPD_TPS_LINE_PWRUP, EPD_TPS_LINE_WAKEUP } epd_tps_line_t;

typedef struct {
    void *ctx;
    esp_err_t (*probe)(void *ctx);
    esp_err_t (*reg_read)(void *ctx, uint8_t reg, uint8_t *val);
    esp_err_t (*reg_write)(void *ctx, uint8_t reg, uint8_t val);
    esp_err_t (*line_set)(void *ctx, epd_tps_line_t line, bool level);
    esp_err_t (*pg_pin)(void *ctx, bool *good);
    void (*oe_low)(void *ctx);              // panel XOE and MODE low (GPIO)
    void (*delay_us)(void *ctx, uint32_t us);
} epd_tps_ops_t;

typedef enum { EPD_TPS_SLEEP, EPD_TPS_STANDBY, EPD_TPS_ON, EPD_TPS_FAILED } epd_tps_state_t;

typedef struct {
    uint32_t pg_ms;         // last power-up: PWRUP to PWR_GOOD pin
    uint32_t pgreg_ms;      // last power-up: ENABLE to PG register good
    uint32_t off_pg_ms;     // last power-down: PWRUP=0 to PWR_GOOD low (EPD_TPS_OFF_PG_MS + 1: stuck)
    uint8_t int1, int2;     // last read (wake, failure or after power-up)
    uint8_t pg_reg;         // last PG register read
    uint32_t on_count, fail_count;
} epd_tps_stats_t;

// Private: use the functions below only
typedef struct {
    const epd_tps_ops_t *ops;
    epd_tps_state_t state;
    uint16_t vcom_mv;
    bool pwrup;             // PWRUP line was driven high since the last power-down
    uint8_t fails;          // consecutive failed power-ups
    epd_tps_stats_t st;
} epd_tps_t;

// Starts in SLEEP. Writes nothing: the board must already hold the three lines low.
esp_err_t epd_tps_init(epd_tps_t *t, const epd_tps_ops_t *ops, uint16_t vcom_mv);
esp_err_t epd_tps_wake(epd_tps_t *t);
esp_err_t epd_tps_hv_on(epd_tps_t *t);
esp_err_t epd_tps_hv_off(epd_tps_t *t);
esp_err_t epd_tps_sleep(epd_tps_t *t);
esp_err_t epd_tps_set_vcom(epd_tps_t *t, uint16_t mv);
esp_err_t epd_tps_temperature(epd_tps_t *t, int8_t *c);
bool epd_tps_pg_ok(epd_tps_t *t);
epd_tps_state_t epd_tps_state(const epd_tps_t *t);
const epd_tps_stats_t *epd_tps_stats(const epd_tps_t *t);
// Diagnostics (STANDBY/ON only). Writes refuse ENABLE and VCOM2 bits other than bit 0.
esp_err_t epd_tps_reg_read(epd_tps_t *t, uint8_t reg, uint8_t *val);
esp_err_t epd_tps_reg_write(epd_tps_t *t, uint8_t reg, uint8_t val);
// VCOM register encoding: magnitude in 10 mV steps (VCOM is negative)
void epd_tps_vcom_encode(uint16_t mv, uint8_t *vcom1, uint8_t *vcom2);
// High-voltage time budget of one refresh: phases x 15 ms + 300 ms, at most 2 s
uint32_t epd_tps_hv_budget_ms(uint16_t phases);

#ifdef __cplusplus
}
#endif
