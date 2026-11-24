# Changelog

## Unreleased

- New example `examples/esp32s3_ssd1677_lvgl`: the minimal v3 pipeline (SSD1677 + `epd_lvgl` L8 + `epd_quant` dithering, synchronous refresh) on the Metalio E-Ink 4. The v2 examples still use the removed glue.
- `epd_ssd1677`: `epd_ssd1677_partial_timing()` / `epd_ssd1677_set_partial_timing()` patch only the TP/RP and frame rate bytes of the stock partial LUT for on-device tuning; timing may only get shorter than stock, VS rows and voltages never change.

## 3.0.0

Breaking: the LVGL 9 RGB565 glue (`epaper_lvgl.c/.h`) is removed and replaced by a new pipeline. The v2 core API and panel drivers (SSD16xx, GDEY0154D67 4-gray, ACeP, BWRY) still build.

- New panel-independent modules, host-tested (`test_host/run.sh`):
  - `epd_quant`: L8 to packed panel planes (1 bpp threshold/Bayer, 2-plane 4-gray, 4 bpp) with rotation, mirroring and invert in one pass.
  - `epd_dirty`: clipped, byte-aligned dirty rectangle set with a full-screen cap.
  - `epd_sched`: refresh scheduler merging LVGL frames and refresh intents, one run in flight, commit reporting, BUSY timeout recovery.
  - `epd_round`: LVGL invalidation rounding on the panel byte axis.
- `epd_bus`: SPI transport with DMA through internal bounce buffers, BUSY interrupt, recording fake for host tests.
- `epd_ssd1677`: SSD1677 controller (800x480 GDEM0397T81-class) with the stock Metalio E-Ink 4 init sequence and waveform tables, LUT cache, full/partial/4-gray modes, deep sleep. Partial refreshes write the whole frame (the ping-pong option keeps the old image).
- `epd_lvgl`: LVGL 9.6 glue, L8 DIRECT full-screen buffer in PSRAM, quantize inside `flush_cb`, `flush_ready` at once; the panel refresh runs on the caller's task.
- Requires ESP-IDF >= 6.1 and LVGL 9.6.0; adds the `esp32s31` target.
- The old examples use the removed glue and no longer build; a v3 example follows.
