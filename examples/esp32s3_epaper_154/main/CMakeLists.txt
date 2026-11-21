idf_component_register(SRCS "app_main.c"
                             "font_vn_12.c"
                             "font_vn_16.c"
                             "font_vn_12_aa.c"
                             "font_vn_16_aa.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_epaper esp_timer)
