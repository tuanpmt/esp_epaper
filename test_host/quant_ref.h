#pragma once

// The row-walk epd_quant_rect before the block path (see quant_ref.c)

#include "epd_quant.h"

void ref_quant_rect(const epd_quant_t *q, const uint8_t *l8, size_t l8_stride, epd_rect_t logical,
                    uint8_t *plane0, uint8_t *plane1, size_t pan_stride);
