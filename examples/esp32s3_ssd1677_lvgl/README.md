# SSD1677 + LVGL 9.6 (esp_epaper v3)

Minimal v3 pipeline on the Metalio E-Ink 4 (ESP32-S3, SSD1677 800x480 panel,
shown as 480x800 portrait):

- LVGL renders L8 into one PSRAM buffer (DIRECT mode, `epd_lvgl_create`).
- `flush_cb` quantizes the dirty areas into a packed 1 bpp plane with
  rotation and ordered dithering (`epd_quant`, `EPD_Q_BW_DITHER`).
- The main loop writes the plane and refreshes the panel synchronously:
  a full clean first, partial refreshes after that, a full clean every
  8 partials.

The screen shows a title, a gray gradient (dithering) and a counter that
changes every 5 s (one partial refresh each).

A real UI should run the refresh on its own task and merge frames with
`epd_sched` while a refresh is in flight; recab-fw's `lvgl_port_epd` does
that, with refresh intents, LUT roles and a ghosting budget.

## Build

```sh
idf.py set-target esp32s3
idf.py build flash monitor
```

Requires ESP-IDF >= 6.1; LVGL 9.6.0 comes from the component manager.
Pins and the TCA9555 screen supply switch are in `menuconfig` under
"SSD1677 LVGL example"; turn the TCA9555 option off on boards whose panel is
always powered.
