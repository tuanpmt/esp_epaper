#pragma once

#include <stdint.h>

#define SSD1677_LUT_SIZE 112
#define SSD1677_LUT_WAVE_BYTES 105

extern const uint8_t ssd1677_lut_fast[SSD1677_LUT_SIZE];      // full clean (0xC7)
extern const uint8_t ssd1677_lut_partial[SSD1677_LUT_SIZE];   // partial / fast (0xCF)
extern const uint8_t ssd1677_lut_gray4[SSD1677_LUT_SIZE];     // 4-gray overlay (0xCC)
