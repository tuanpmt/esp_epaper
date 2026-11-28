// Minimal esp_epaper v3 example: LVGL 9.6 on an SSD1677 800x480 panel
// (Metalio E-Ink 4), shown as 480x800 portrait.
//
// LVGL renders L8 into one PSRAM buffer; epd_lvgl quantizes the dirty areas
// into a packed 1 bpp panel plane inside flush_cb and reports the frame with
// on_frame(). This example then refreshes the panel synchronously on the same
// task: a full clean for the first frame, partial refreshes after that, and a
// full clean every CLEAN_EVERY partials against ghosting. A real UI runs the
// refresh on its own task with epd_sched (merging frames while one refresh
// is in flight); see README.md.

#include <string.h>
#include "driver/i2c_master.h"
#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "epd_bus.h"
#include "epd_lvgl.h"
#include "epd_quant.h"
#include "epd_ssd1677.h"
#include "sdkconfig.h"

static const char *TAG = "ssd1677_lvgl";

#define PAN_W 800           // native landscape panel
#define PAN_H 480
#define LOG_W 480           // LVGL portrait
#define LOG_H 800
#define STRIDE (PAN_W / 8)  // 1 bpp panel row
#define CLEAN_EVERY 8       // partial refreshes between two full cleans

static epd_ssd1677_t *s_epd;
static epd_quant_t s_quant;
static uint8_t *s_plane;
static SemaphoreHandle_t s_done;
static volatile bool s_frame;   // LVGL quantized a frame since the last refresh

#if CONFIG_EXAMPLE_SCREEN_PWR_TCA9555
// Drive one TCA9555 pin high as an output: the Metalio panel socket supply
static esp_err_t screen_power_on(void)
{
    const i2c_master_bus_config_t bc = {
        .i2c_port = -1, .sda_io_num = CONFIG_EXAMPLE_TCA_SDA, .scl_io_num = CONFIG_EXAMPLE_TCA_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bc, &bus), TAG, "i2c bus");
    const i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x20, .scl_speed_hz = 100000};
    i2c_master_dev_handle_t dev;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dc, &dev), TAG, "tca9555");
    const uint8_t port = CONFIG_EXAMPLE_TCA_PIN / 8, bit = 1u << (CONFIG_EXAMPLE_TCA_PIN % 8);
    uint8_t reg = 0x02 + port, val;     // output port: set the bit first
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 100), TAG, "read out");
    uint8_t w[2] = {reg, (uint8_t)(val | bit)};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, w, 2, 100), TAG, "write out");
    reg = 0x06 + port;                  // configuration: 0 = output
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 100), TAG, "read cfg");
    w[0] = reg;
    w[1] = (uint8_t)(val & ~bit);
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, w, 2, 100), TAG, "write cfg");
    vTaskDelay(pdMS_TO_TICKS(20));      // let the supply settle before the panel reset
    return ESP_OK;
}
#endif

// BUSY dropped: from the GPIO ISR, or from the caller when the panel was idle
static void busy_done(void *ctx)
{
    (void)ctx;
    if (xPortInIsrContext()) {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_done, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xSemaphoreGive(s_done);
    }
}

// LVGL task: the frame's areas are already in s_plane
static void on_frame(void *ctx, const epd_rect_t *panel_rects, uint8_t n)
{
    (void)ctx;
    (void)panel_rects;
    s_frame = n > 0;
}

// Send the whole plane (partial windows are unreliable in the SSD1677's
// ping-pong mode, so the frame is always written in full) and refresh
static esp_err_t refresh(epd_mode_t mode)
{
    const epd_rect_t all = {0, 0, PAN_W, PAN_H};
    ESP_RETURN_ON_ERROR(epd_ssd1677_write(s_epd, EPD_SSD1677_RAM_NEW, all, s_plane, STRIDE), TAG, "write");
    if (mode == EPD_M_FULL_CLEAN) {
        ESP_RETURN_ON_ERROR(epd_ssd1677_write(s_epd, EPD_SSD1677_RAM_OLD, all, s_plane, STRIDE), TAG, "write old");
    }
    const int64_t t0 = esp_timer_get_time();
    ESP_RETURN_ON_ERROR(epd_ssd1677_start(s_epd, mode, busy_done, NULL), TAG, "start");
    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "BUSY timeout, re-init");
        return epd_ssd1677_reinit(s_epd);
    }
    ESP_LOGI(TAG, "%s refresh %lld ms", mode == EPD_M_FULL_CLEAN ? "full" : "partial",
             (long long)((esp_timer_get_time() - t0) / 1000));
    return epd_ssd1677_after(s_epd, mode, all, s_plane, STRIDE);
}

static uint32_t tick_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static lv_obj_t *s_count;

static void count_cb(lv_timer_t *t)
{
    (void)t;
    static unsigned n;
    lv_label_set_text_fmt(s_count, "%u", ++n);
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_label_set_text(title, "esp_epaper v3\nSSD1677 + LVGL 9.6");
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 80);

    // A gray gradient shows the ordered dithering of EPD_Q_BW_DITHER
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_set_size(bar, 400, 80);
    lv_obj_set_style_bg_color(bar, lv_color_black(), 0);
    lv_obj_set_style_bg_grad_color(bar, lv_color_white(), 0);
    lv_obj_set_style_bg_grad_dir(bar, LV_GRAD_DIR_HOR, 0);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, -60);

    s_count = lv_label_create(scr);
    lv_obj_set_style_text_font(s_count, &lv_font_montserrat_48, 0);
    lv_label_set_text(s_count, "0");
    lv_obj_align(s_count, LV_ALIGN_CENTER, 0, 120);
    lv_timer_create(count_cb, 5000, NULL);
}

void app_main(void)
{
#if CONFIG_EXAMPLE_SCREEN_PWR_TCA9555
    ESP_ERROR_CHECK(screen_power_on());
#endif
    s_done = xSemaphoreCreateBinary();
    s_plane = heap_caps_malloc(STRIDE * PAN_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(s_done && s_plane);
    memset(s_plane, 0xFF, STRIDE * PAN_H);  // white

    const epd_bus_spi_cfg_t bc = {
        .host = SPI3_HOST, .mosi = CONFIG_EXAMPLE_EPD_MOSI, .sclk = CONFIG_EXAMPLE_EPD_SCLK,
        .cs = CONFIG_EXAMPLE_EPD_CS, .dc = CONFIG_EXAMPLE_EPD_DC, .rst = CONFIG_EXAMPLE_EPD_RST,
        .busy = CONFIG_EXAMPLE_EPD_BUSY, .hz = 10000000, .busy_active_high = true, .bounce = 4096,
    };
    epd_bus_t *bus;
    ESP_ERROR_CHECK(epd_bus_spi_new(&bc, &bus));
    const epd_ssd1677_cfg_t cc = {.bus = bus, .pan_w = PAN_W, .pan_h = PAN_H, .auto_old_plane = true};
    ESP_ERROR_CHECK(epd_ssd1677_init(&cc, &s_epd));

    // Portrait LVGL on the landscape panel: rotate 90 degrees, dither grays
    epd_quant_init(&s_quant, LOG_W, LOG_H, PAN_W, PAN_H, EPD_ROT_90);
    epd_quant_set_mode(&s_quant, EPD_Q_BW_DITHER, false);

    lv_init();
    lv_tick_set_cb(tick_ms);
    const epd_lvgl_cfg_t gc = {
        .hor_res = LOG_W, .ver_res = LOG_H, .quant = &s_quant, .on_frame = on_frame, .ctx = NULL,
        .panel_plane0 = s_plane, .panel_plane1 = NULL, .pan_stride = STRIDE, .align = 8, .pad_x = 24, .pad_y = 12,
    };
    lv_display_t *disp = epd_lvgl_create(&gc);
    assert(disp);
    build_ui();

    unsigned partials = 0;
    bool first = true;
    for (;;) {
        const uint32_t wait = lv_timer_handler();
        if (s_frame) {
            s_frame = false;
            const bool clean = first || partials >= CLEAN_EVERY;
            ESP_ERROR_CHECK(refresh(clean ? EPD_M_FULL_CLEAN : EPD_M_PARTIAL));
            partials = clean ? 0 : partials + 1;
            first = false;
            epd_lvgl_stats_t st;
            epd_lvgl_stats(disp, &st);
            ESP_LOGI(TAG, "render %lu us, quantize %lu us, %lu px", (unsigned long)st.render_us,
                     (unsigned long)st.quant_us, (unsigned long)st.last_area_px);
        }
        vTaskDelay(pdMS_TO_TICKS(wait < 10 ? 10 : (wait > 500 ? 500 : wait)));
    }
}
