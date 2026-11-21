set(EPD_TARGET_SRCS "")
set(EPD_TARGET_PRIV "")
if(IDF_TARGET STREQUAL "esp32s31")
    # parallel panel scan bus (LCD_CAM + RMT), ESP32-S31 only
    list(APPEND EPD_TARGET_SRCS "src/bus/epd_par_bus_s31.c")
    list(APPEND EPD_TARGET_PRIV esp_driver_rmt esp_driver_dma esp_hal_lcd esp_hal_i2s esp_mm esp_hw_support esp_hal_gpio)
endif()

idf_component_register(
    SRCS
        ${EPD_TARGET_SRCS}
        "src/epaper.c"
        "src/epaper_spi.c"
        "src/epd_bus_spi.c"
        "src/epaper_common.c"
        "src/epaper_registry.c"
        "src/epd_sched.c"
        "src/epd_quant.c"
        "src/epd_dirty.c"
        "src/epd_round.c"
        "src/epd_par_timing.c"
        "src/epd_wf.c"
        "src/epd_wf_mt.c"
        "src/epd_par_run.c"
        "src/panels/e0470_wf_data.c"
        "src/epd_lvgl.c"
        "src/controllers/ssd1677.c"
        "src/controllers/ssd1677_lut.c"
        "src/controllers/tps65185.c"
        "src/controllers/ssd16xx.c"
        "src/controllers/gdey0154_lut.c"
        "src/controllers/acep_6color.c"
        "src/controllers/bwry_4color.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_timer
        lvgl
    PRIV_REQUIRES
        ${EPD_TARGET_PRIV}
        esp_psram
        log
)

# The waveform inner loop runs once per panel byte per phase (85 Hz): optimise
# it even in debug (-Og) builds; the last -O on the command line wins
set_source_files_properties("src/epd_wf.c" "src/epd_wf_mt.c" PROPERTIES COMPILE_OPTIONS "-O2")
