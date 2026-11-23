#pragma once

#include <stdint.h>

#define SSD1677_LUT_SIZE 112
#define SSD1677_LUT_WAVE_BYTES 105

extern const uint8_t ssd1677_lut_fast[SSD1677_LUT_SIZE];      // full clean (0xC7)
extern const uint8_t ssd1677_lut_partial[SSD1677_LUT_SIZE];   // partial / fast (0xCF)
extern const uint8_t ssd1677_lut_gray4[SSD1677_LUT_SIZE];     // 4-gray overlay (0xCC)

// Timing candidate (epd_ssd1677_lut_candidate) loaded at init into the
// partial table (PARTIAL, TRANSITION) and the fast table (FAST)
extern const uint8_t ssd1677_lut_pick_partial;
extern const uint8_t ssd1677_lut_pick_fast;
