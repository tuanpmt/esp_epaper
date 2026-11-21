// ESP32-S31 scan bus: LCD_CAM RGB + RMT CKV + GPIO SPV (see epd_par_bus_s31.h).

#include <string.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "esp_private/gpio.h"
#include "esp_private/periph_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/gpio_ll.h"
#include "hal/lcd_ll.h"
#include "hal/lcd_periph.h"
#include "soc/gpio_sig_map.h"
#include "epd_par_bus_s31.h"

static const char *TAG = "epd_s31_bus";

#define DMA_NODE_BYTES 4032     // < 4095, a multiple of the 64-byte cache line
#define EDGE_TIMEOUT_LINES 2

static struct {
    bool ready;
    epd_s31_bus_cfg_t cfg;
    epd_par_timing_t t;
    lcd_cam_dev_t *dev;
    intr_handle_t intr;
    gdma_channel_handle_t dma;
    gdma_link_list_handle_t link[2];
    uint8_t *buf[2];
    size_t bytes;
    rmt_channel_handle_t rmt;
    rmt_encoder_handle_t enc;
    rmt_symbol_word_t sym;
    SemaphoreHandle_t done;
    portMUX_TYPE mux;
    uint32_t cycles_per_us, tick_hz;
    int64_t t_lcd_start;
    volatile int64_t t_vsync, t_rmt_done;
    volatile bool underrun;
    volatile uint32_t eofs;
    epd_s31_bus_stats_t st;
} s = {.mux = portMUX_INITIALIZER_UNLOCKED};

static void lcd_isr(void *arg)
{
    (void)arg;
    const uint32_t st = lcd_ll_get_interrupt_status(s.dev);
    lcd_ll_clear_interrupt_status(s.dev, st);
    BaseType_t woken = pdFALSE;
    if (st & LCD_LL_EVENT_UNDERRUN) {
        s.underrun = true;
    }
    if (st & LCD_LL_EVENT_VSYNC_END) {
        s.t_vsync = esp_timer_get_time();
        xSemaphoreGiveFromISR(s.done, &woken);
    }
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

static IRAM_ATTR bool on_dma_eof(gdma_channel_handle_t ch, gdma_event_data_t *ev, void *user)
{
    (void)ch;
    (void)ev;
    (void)user;
    s.eofs++;
    return false;
}

static IRAM_ATTR bool on_rmt_done(rmt_channel_handle_t ch, const rmt_tx_done_event_data_t *ev, void *user)
{
    (void)ch;
    (void)ev;
    (void)user;
    s.t_rmt_done = esp_timer_get_time();
    return false;
}

// ---- pads: routed only while a frame runs ----

static void route(bool on)
{
    const epd_s31_bus_cfg_t *c = &s.cfg;
    const soc_lcd_rgb_signal_desc_t *sig = &soc_lcd_rgb_signals[0];
    const soc_lcd_rgb_iomux_desc_t *mux = &soc_lcd_rgb_iomux_descs[0];
    for (int i = 0; i < 8; i++) {
        const int pin = c->data0 + i;
        if (on && mux->data_pins[i].gpio_num == pin) {
            gpio_iomux_output(pin, mux->data_pins[i].func);    // dedicated pads: least skew
        } else if (on) {
            gpio_matrix_output(pin, sig->data_sigs[i], false, false);
        } else {
            gpio_set_level(pin, 0);
            esp_rom_gpio_pad_select_gpio(pin);
            gpio_matrix_output(pin, SIG_GPIO_OUT_IDX, false, false);
        }
    }
    const int pins[3] = {c->xcl, c->xle, c->xstl};
    const int sigs[3] = {sig->pclk_sig, sig->hsync_sig, sig->de_sig};
    const int idle[3] = {0, c->xle_active_low ? 1 : 0, c->xstl_active_high ? 0 : 1};   // XSTL rests high
    for (int i = 0; i < 3; i++) {
        if (pins[i] < 0) {
            continue;
        }
        if (on) {
            gpio_matrix_output(pins[i], sigs[i], false, false);
        } else {
            gpio_set_level(pins[i], idle[i]);
            gpio_matrix_output(pins[i], SIG_GPIO_OUT_IDX, false, false);
        }
    }
}

static void oe(bool on)
{
    if (s.cfg.xoe >= 0) {
        gpio_set_level(s.cfg.xoe, on);
    }
    if (s.cfg.mode >= 0) {
        gpio_set_level(s.cfg.mode, on);
    }
}

// ---- frame start, timed ----

static IRAM_ATTR bool wait_ckv(int level, uint32_t limit_cycles, uint32_t *at)
{
    const uint32_t t0 = esp_cpu_get_cycle_count();
    while (gpio_ll_get_level(&GPIO, s.cfg.ckv) != level) {
        if (esp_cpu_get_cycle_count() - t0 > limit_cycles) {
            return false;
        }
    }
    *at = esp_cpu_get_cycle_count();
    return true;
}

static IRAM_ATTR void delay_cycles(uint32_t cycles)
{
    const uint32_t t0 = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - t0 < cycles) {
    }
}

// Returns false on a missed CKV edge. *lead = cycles from t_rmt to the 2nd rising edge.
static IRAM_ATTR bool start_locked(uint32_t t_rmt, uint32_t *lead)
{
    const uint32_t line = s.t.line_ns * s.cycles_per_us / 1000u;
    const uint32_t limit = EDGE_TIMEOUT_LINES * line;
    const uint32_t extra = (uint32_t)s.cfg.start_delay_ns * s.cycles_per_us / 1000u;
    uint32_t at = 0;
    if (!s.cfg.route_pins) {
        // Internal check: no pad to watch. Start where the 2nd falling edge would be.
        const uint32_t high = (uint32_t)((uint64_t)line * s.t.ckv_high_ticks / s.t.ckv_period_ticks);
        const uint32_t target = line + high + extra;
        const uint32_t spent = esp_cpu_get_cycle_count() - t_rmt;
        if (spent < target) {
            delay_cycles(target - spent);
        }
        lcd_ll_start(s.dev);
        *lead = line;
        return true;
    }
    if (!wait_ckv(0, limit, &at) || !wait_ckv(1, limit, &at)) {   // 1st falling, 2nd rising
        return false;
    }
    *lead = at - t_rmt;
    gpio_ll_set_level(&GPIO, s.cfg.spv, 1);     // SPV was low across the 1st rising edge
    if (!wait_ckv(0, limit, &at)) {             // 2nd falling: the LCD frame starts here
        return false;
    }
    delay_cycles(extra);
    lcd_ll_start(s.dev);
    return true;
}

static esp_err_t scan_start(void *ctx, const uint8_t *buf)
{
    (void)ctx;
    const int idx = buf == s.buf[0] ? 0 : buf == s.buf[1] ? 1 : -1;
    if (!s.ready || idx < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s.cfg.route_pins) {
        gpio_set_level(s.cfg.spv, 0);
        route(true);
        if (s.cfg.drive_oe) {
            oe(true);   // every frame: hv_off lowers XOE/MODE behind the bus's back
        }
    }
    lcd_ll_stop(s.dev);
    lcd_ll_reset(s.dev);
    lcd_ll_fifo_reset(s.dev);
    gdma_reset(s.dma);
    lcd_ll_clear_interrupt_status(s.dev, UINT32_MAX);
    s.underrun = false;
    s.eofs = 0;
    s.t_rmt_done = 0;
    xSemaphoreTake(s.done, 0);
    gdma_start(s.dma, gdma_link_get_head_addr(s.link[idx]));
    esp_rom_delay_us(1);    // DMA fills the LCD FIFO before the engine starts
    s.cycles_per_us = (uint32_t)(esp_clk_cpu_freq() / 1000000);   // DFS may have moved it since setup
    const rmt_transmit_config_t tc = {.loop_count = s.t.frame_lines + 1, .flags.eot_level = 0};
    const uint32_t t_rmt = esp_cpu_get_cycle_count();
    esp_err_t err = rmt_transmit(s.rmt, s.enc, &s.sym, sizeof(s.sym), &tc);
    uint32_t lead = 0;
    bool ok = false;
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s.mux);
        ok = start_locked(t_rmt, &lead);
        portEXIT_CRITICAL(&s.mux);
        s.t_lcd_start = esp_timer_get_time();
    }
    s.st.last_lead_ns = lead * 1000u / s.cycles_per_us;
    const uint32_t line = s.t.line_ns;
    if (err == ESP_OK && (!ok || s.st.last_lead_ns < line * 8 / 10 || s.st.last_lead_ns > line * 19 / 10)) {
        err = ESP_ERR_INVALID_STATE;    // CKV and the LCD would be off by a line: never scan like that
        s.st.sync_errors++;
    }
    if (err != ESP_OK) {
        oe(false);
        lcd_ll_stop(s.dev);
        gdma_stop(s.dma);
        rmt_disable(s.rmt);
        rmt_enable(s.rmt);
        if (s.cfg.route_pins) {
            route(false);
            gpio_set_level(s.cfg.spv, 1);   // SPV rests high (active low)
        }
    }
    return err;
}

static esp_err_t scan_wait(void *ctx, uint32_t timeout_us)
{
    (void)ctx;
    const TickType_t ticks = (TickType_t)(((uint64_t)timeout_us * configTICK_RATE_HZ + 999999u) / 1000000u) + 1;
    esp_err_t err = xSemaphoreTake(s.done, ticks) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
    if (err == ESP_OK) {
        rmt_tx_wait_all_done(s.rmt, 5);     // CKV ends about one line after the LCD frame
    } else {
        s.st.timeouts++;
        oe(false);
        rmt_disable(s.rmt);
        rmt_enable(s.rmt);
    }
    lcd_ll_stop(s.dev);
    gdma_stop(s.dma);
    if (s.cfg.route_pins) {
        route(false);
        gpio_set_level(s.cfg.spv, 1);   // SPV rests high (active low)
    }
    if (s.underrun) {
        s.st.underruns++;
        oe(false);
        err = ESP_ERR_INVALID_STATE;
    }
    s.st.last_dma_eofs = s.eofs;
    if (err == ESP_OK) {
        s.st.frames++;
        s.st.last_frame_us = (uint32_t)(s.t_vsync - s.t_lcd_start);
        s.st.last_rmt_lag_us = s.t_rmt_done ? (int32_t)(s.t_rmt_done - s.t_vsync) : INT32_MIN;
    }
    return err;
}

static void park(void *ctx)
{
    (void)ctx;
    oe(false);
    if (!s.ready) {
        return;
    }
    lcd_ll_stop(s.dev);
    gdma_stop(s.dma);
    if (s.cfg.route_pins) {
        route(false);   // SPV untouched: the PMIC side sets the idle levels (all low in SLEEP)
    }
}

// ---- setup ----

static esp_err_t lcd_setup(const epd_par_panel_t *p)
{
    PERIPH_RCC_ACQUIRE_ATOMIC(soc_lcd_rgb_signals[0].module, ref_count) {
        if (ref_count == 0) {
            lcd_ll_enable_bus_clock(0, true);
            lcd_ll_reset_register(0);
            lcd_ll_select_core_clk_src(0, LCD_CORE_CLK_SRC_DEFAULT);
            lcd_ll_set_core_clock_divider(0, 2, 0, 0);
        }
    }
    s.dev = LCD_LL_GET_HW(0);
    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src((soc_module_clk_t)LCD_CORE_CLK_SRC_DEFAULT, true), TAG, "core clk");
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_clock(s.dev, true);
    }
    lcd_ll_mem_set_low_power_mode(s.dev, LCD_LL_MEM_LP_MODE_SHUT_DOWN);
    lcd_ll_mem_power_by_pmu(s.dev);
    lcd_ll_enable_trans_buffer(s.dev, true);
    uint32_t src_hz = 0;
    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src((soc_module_clk_t)LCD_CLK_SRC_PLL160M, true), TAG, "lcd clk");
    ESP_RETURN_ON_ERROR(esp_clk_tree_src_get_freq_hz((soc_module_clk_t)LCD_CLK_SRC_PLL160M,
                                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz), TAG, "freq");
    PERIPH_RCC_ATOMIC() {
        lcd_ll_select_clk_src(0, LCD_CLK_SRC_PLL160M);
    }
    lcd_ll_reset(s.dev);
    lcd_ll_fifo_reset(s.dev);
    // pclk = src / div / 2, an exact integer division (no fractional jitter on XCL)
    const uint32_t div = src_hz / (p->pclk_hz * 2u);
    ESP_RETURN_ON_FALSE(div >= 2 && div <= LCD_LL_CLK_FRAC_DIV_N_MAX && div * p->pclk_hz * 2u == src_hz,
                        ESP_ERR_INVALID_ARG, TAG, "pclk %u Hz from %u Hz", (unsigned)p->pclk_hz, (unsigned)src_hz);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_set_group_clock_coeff(0, (int)div, 0, 0);
    }
    lcd_ll_set_pixel_clock_prescale(s.dev, 2);
    lcd_ll_set_clock_idle_level(s.dev, false);
    lcd_ll_set_pixel_clock_edge(s.dev, s.cfg.xcl_out_neg);
    lcd_ll_enable_rgb_mode(s.dev, true);
    lcd_ll_set_dma_read_stride(s.dev, 8);
    lcd_ll_set_data_wire_width(s.dev, 8);
    lcd_ll_enable_color_convert(s.dev, false);
    lcd_ll_set_phase_cycles(s.dev, 0, 0, 1);
    lcd_ll_enable_output_always_on(s.dev, true);
    lcd_ll_set_idle_level(s.dev, s.cfg.xle_active_low, false, !s.cfg.xstl_active_high);
    lcd_ll_set_blank_cycles(s.dev, 1, 1);
    lcd_ll_set_horizontal_timing(s.dev, p->le_high_clk, p->front_porch_clk, s.t.line_bytes, s.t.h_front);
    const uint32_t rows = s.t.frame_lines - 1u - s.cfg.v_back - s.cfg.v_front;
    lcd_ll_set_vertical_timing(s.dev, 1, s.cfg.v_back, rows, s.cfg.v_front);
    lcd_ll_enable_output_hsync_in_porch_region(s.dev, true);
    lcd_ll_set_hsync_position(s.dev, 0);
    lcd_ll_enable_auto_next_frame(s.dev, false);   // one frame per start
    ESP_RETURN_ON_ERROR(esp_intr_alloc_intrstatus(soc_lcd_rgb_signals[0].irq_id,
                                                  ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED,
                                                  (uint32_t)lcd_ll_get_interrupt_status_reg(s.dev), LCD_LL_EVENT_RGB,
                                                  lcd_isr, NULL, &s.intr), TAG, "intr");
    lcd_ll_clear_interrupt_status(s.dev, UINT32_MAX);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(s.dev, LCD_LL_EVENT_RGB, true);
    }
    return ESP_OK;
}

static esp_err_t dma_setup(void)
{
    const gdma_channel_alloc_config_t cc = {0};
    ESP_RETURN_ON_ERROR(gdma_new_axi_channel(&cc, &s.dma, NULL), TAG, "dma channel");
    gdma_connect(s.dma, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    const gdma_strategy_config_t strat = {.eof_till_data_popped = false};
    gdma_apply_strategy(s.dma, &strat);
    const gdma_transfer_config_t tc = {.max_data_burst_size = 32, .access_ext_mem = true};
    ESP_RETURN_ON_ERROR(gdma_config_transfer(s.dma, &tc), TAG, "dma transfer");
    gdma_channel_alignment_info_t al;
    gdma_get_channel_alignment_constraints(s.dma, &al);
    for (int i = 0; i < 2; i++) {
        const size_t align = esp_ptr_external_ram(s.buf[i]) ? al.ext_enc_mem_alignment : al.int_mem_alignment;
        const gdma_link_list_config_t lc = {.num_items = s.bytes / DMA_NODE_BYTES + 2, .item_alignment = 8,
                                            .flags.check_owner = false};
        ESP_RETURN_ON_ERROR(gdma_new_link_list(&lc, &s.link[i]), TAG, "dma link");
        const gdma_buffer_mount_config_t mc = {
            .buffer = s.buf[i], .buffer_alignment = align ? align : 1, .length = s.bytes,
            .flags = {.mark_eof = 1, .mark_final = GDMA_FINAL_LINK_TO_NULL},
        };
        ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(s.link[i], 0, &mc, 1, NULL), TAG, "dma mount %d", i);
    }
    const gdma_tx_event_callbacks_t cbs = {.on_trans_eof = on_dma_eof};
    return gdma_register_tx_event_callbacks(s.dma, (gdma_tx_event_callbacks_t *)&cbs, NULL);
}

static esp_err_t rmt_setup(void)
{
    const rmt_tx_channel_config_t c = {
        .gpio_num = s.cfg.route_pins ? s.cfg.ckv : -1,
        .clk_src = RMT_CLK_SRC_DEFAULT,     // REF_F80M / 8 = 10 MHz: same crystal as the LCD's PLL, no drift
        .resolution_hz = s.tick_hz,
        .mem_block_symbols = 48,
        .trans_queue_depth = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&c, &s.rmt), TAG, "rmt channel");
    const rmt_copy_encoder_config_t ec = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&ec, &s.enc), TAG, "rmt encoder");
    const rmt_tx_event_callbacks_t cbs = {.on_trans_done = on_rmt_done};
    ESP_RETURN_ON_ERROR(rmt_tx_register_event_callbacks(s.rmt, &cbs, NULL), TAG, "rmt cb");
    ESP_RETURN_ON_ERROR(rmt_enable(s.rmt), TAG, "rmt enable");
    s.sym = (rmt_symbol_word_t){.level0 = 1, .duration0 = s.t.ckv_high_ticks, .level1 = 0,
                                .duration1 = s.t.ckv_low_ticks};
    if (s.cfg.route_pins) {
        gpio_input_enable(s.cfg.ckv);   // the frame start watches the CKV pad
    }
    return ESP_OK;
}

esp_err_t epd_s31_bus_new(const epd_s31_bus_cfg_t *cfg, const epd_par_panel_t *panel, const epd_par_timing_t *t,
                          uint8_t *const bufs[2], epd_par_bus_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && panel && t && bufs && bufs[0] && bufs[1] && out, ESP_ERR_INVALID_ARG, TAG, "args");
    ESP_RETURN_ON_FALSE(!s.ready, ESP_ERR_INVALID_STATE, TAG, "already created");
    ESP_RETURN_ON_FALSE(1u + cfg->v_back + cfg->v_front + panel->rows == t->frame_lines, ESP_ERR_INVALID_ARG, TAG,
                        "1 + %u + %u + %u lines != frame %u", cfg->v_back, cfg->v_front, panel->rows, t->frame_lines);
    s.cfg = *cfg;
    s.t = *t;
    s.buf[0] = bufs[0];
    s.buf[1] = bufs[1];
    s.bytes = (size_t)t->line_bytes * panel->rows;
    s.cycles_per_us = (uint32_t)(esp_clk_cpu_freq() / 1000000);
    s.tick_hz = panel->ckv_tick_hz;
    s.done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s.done, ESP_ERR_NO_MEM, TAG, "sem");
    // Pads stay GPIO outputs at their current levels until a frame routes
    // them: the PMIC sequencer sets the idle levels (SPV/XSTL high while powered)
    ESP_RETURN_ON_ERROR(lcd_setup(panel), TAG, "lcd");
    ESP_RETURN_ON_ERROR(dma_setup(), TAG, "dma");
    ESP_RETURN_ON_ERROR(rmt_setup(), TAG, "rmt");
    s.ready = true;
    *out = (epd_par_bus_t){.ctx = NULL, .scan_start = scan_start, .scan_wait = scan_wait, .park = park};
    ESP_LOGI(TAG, "ready: %s, %u lines of %u bytes, ckv %u/%u ticks, pclk %u Hz", cfg->route_pins ? "pins" : "internal",
             (unsigned)t->frame_lines, (unsigned)t->line_bytes, (unsigned)t->ckv_high_ticks,
             (unsigned)t->ckv_low_ticks, (unsigned)panel->pclk_hz);
    return ESP_OK;
}

void epd_s31_bus_del(void)
{
    if (!s.ready) {
        return;
    }
    park(NULL);
    rmt_disable(s.rmt);
    rmt_del_channel(s.rmt);
    rmt_del_encoder(s.enc);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(s.dev, LCD_LL_EVENT_RGB, false);
    }
    esp_intr_free(s.intr);
    gdma_disconnect(s.dma);
    gdma_del_channel(s.dma);
    gdma_del_link_list(s.link[0]);
    gdma_del_link_list(s.link[1]);
    vSemaphoreDelete(s.done);
    const portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
    memset(&s, 0, sizeof(s));
    s.mux = m;
}

void epd_s31_bus_stats(epd_s31_bus_stats_t *st)
{
    *st = s.st;
}

esp_err_t epd_s31_bus_flush(const uint8_t *buf, uint16_t y0, uint16_t rows)
{
    uint8_t *p = (uint8_t *)buf + (size_t)y0 * s.t.line_bytes;
    if (esp_cache_get_line_size_by_addr(p) == 0) {
        return ESP_OK;  // not behind a cache
    }
    return esp_cache_msync(p, (size_t)rows * s.t.line_bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}
