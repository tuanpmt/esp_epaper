// ESP32-S31 scan bus: LCD_CAM RGB + RMT CKV + GPIO SPV (see epd_par_bus_s31.h).
//
// Own code against the ESP-IDF v6.1.1 LL headers. It reproduces the signals
// and the order of the vendor firmware of the Metalio E-Ink 4 Plus, whose
// display/output_lcd/lcd_driver.c is EPDiy-derived (LGPL): behaviour only,
// nothing copied. Every place that mirrors it cites "stock lcd_driver.c:NNN".

#include <string.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_check.h"
#include "esp_clk_tree.h"
#include "esp_cpu.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
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
#include "hal/rmt_ll.h"
#include "hal/rmt_periph.h"
#include "hal/rmt_types.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_struct.h"
#include "epd_par_bus_s31.h"

static const char *TAG = "epd_s31_bus";

#define CKV_CHAN 1              // stock lcd_driver.c:125 (RMT_CHANNEL_1)
#define CKV_TICK_HZ 10000000u   // stock :393-406: REF_F80M / 8
#define CKV_EXTRA_LINES 5       // stock :1324: start_ckv_cycles(lines + 5)
#define DMA_NODE_BYTES 4032     // < 4095, a multiple of the 64-byte cache line
#define DMA_BURST 64            // stock :601
#define LCD_EVENTS (LCD_LL_EVENT_VSYNC_END | LCD_LL_EVENT_TRANS_DONE)   // stock :907, :1325

// RMT channel memory (linker symbol, soc/esp32s31/ld): 48 words per channel
extern volatile rmt_symbol_word_t RMTMEM[];

static struct {
    bool ready;
    bool routed;                // pads on the peripherals for the current refresh
    epd_s31_bus_cfg_t cfg;
    epd_par_timing_t t;
    uint16_t rows;
    uint8_t le_high_clk, front_porch_clk;
    lcd_cam_dev_t *dev;
    intr_handle_t intr;
    gdma_channel_handle_t dma;
    gdma_link_list_handle_t link[2];
    uint8_t *buf[2];
    size_t bytes;
    SemaphoreHandle_t done;
    portMUX_TYPE mux;
    uint32_t ckv_period_us, ckv_high_us;   // the integer delays of the frame start
    int64_t t_lcd_start, t_rmt_start;
    volatile int64_t t_vsync, t_eof;
    volatile uint32_t eofs, underrun_seen;
    epd_s31_bus_stats_t st;
} s = {.mux = portMUX_INITIALIZER_UNLOCKED};

// ---- interrupts ----

// The LL helper is a macro restricted to PERIPH_RCC_ATOMIC in IDF 6.1.1; this
// is its body (lcd_ll.h: lc_dma_int_ena, bits 0x13), usable from the ISR and
// the frame-start critical section
static IRAM_ATTR void int_enable(bool on)
{
    const uint32_t mask = (LCD_EVENTS | LCD_LL_EVENT_UNDERRUN) & 0x13u;
    if (on) {
        s.dev->lc_dma_int_ena.val |= mask;
    } else {
        s.dev->lc_dma_int_ena.val &= ~mask;
    }
}

// stock lcd_driver.c:470-509 (single batch): VSYNC_END ends the frame; stop
// the LCD and the DMA (stock :247-266)
static IRAM_ATTR void lcd_isr(void *arg)
{
    (void)arg;
    const uint32_t st = lcd_ll_get_interrupt_status(s.dev);
    lcd_ll_clear_interrupt_status(s.dev, st);
    if (st & LCD_LL_EVENT_UNDERRUN) {
        s.underrun_seen++;      // not in stock: a guard for the PSRAM DMA source
    }
    if (st & LCD_LL_EVENT_VSYNC_END) {
        int_enable(false);
        lcd_ll_stop(s.dev);
        s.t_vsync = esp_timer_get_time();
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s.done, &woken);
        if (woken) {
            portYIELD_FROM_ISR();
        }
    }
}

static IRAM_ATTR bool on_dma_eof(gdma_channel_handle_t ch, gdma_event_data_t *ev, void *user)
{
    (void)ch;
    (void)ev;
    (void)user;
    s.eofs++;
    s.t_eof = esp_timer_get_time();     // the last data byte entered the LCD
    return false;
}

// ---- pads ----
// stock :663-722 routes data (IOMUX only, :665-670), XLE = HSYNC, XCL = PCLK,
// XSTL = DE (GPIO matrix) permanently and SPV as a GPIO initialised high. We
// route them only while a frame runs; between frames the pads are GPIOs at
// the same levels the idle LCD outputs (data, XCL, XLE low; XSTL high).

static void route(bool on)
{
    const epd_s31_bus_cfg_t *c = &s.cfg;
    const soc_lcd_rgb_signal_desc_t *sig = &soc_lcd_rgb_signals[0];
    const soc_lcd_rgb_iomux_desc_t *mux = &soc_lcd_rgb_iomux_descs[0];
    for (int i = 0; i < 8; i++) {
        const int pin = c->data0 + i;
        if (on) {
            gpio_iomux_output(pin, mux->data_pins[i].func);
        } else {
            gpio_set_level(pin, 0);
            esp_rom_gpio_pad_select_gpio(pin);
            gpio_matrix_output(pin, SIG_GPIO_OUT_IDX, false, false);
        }
    }
    const int pins[3] = {c->xcl, c->xle, c->xstl};
    const int sigs[3] = {sig->pclk_sig, sig->hsync_sig, sig->de_sig};
    const int idle[3] = {0, 0, 1};      // stock :962: HSYNC idle low, DE idle high; PCLK idle low (:937)
    for (int i = 0; i < 3; i++) {
        if (on) {
            gpio_matrix_output(pins[i], sigs[i], false, false);
        } else {
            gpio_set_level(pins[i], idle[i]);
            gpio_matrix_output(pins[i], SIG_GPIO_OUT_IDX, false, false);
        }
    }
    if (on) {   // stock :437-444: CKV pad from the RMT channel
        esp_rom_gpio_connect_out_signal(c->ckv, soc_rmt_signals[0].channels[CKV_CHAN].tx_sig, false, false);
    } else {
        gpio_set_level(c->ckv, 0);
        gpio_matrix_output(c->ckv, SIG_GPIO_OUT_IDX, false, false);
    }
}

static void oe(bool on)
{
    if (on && !s.cfg.drive_oe) {
        return;
    }
    gpio_set_level(s.cfg.xoe, on);
    gpio_set_level(s.cfg.mode, on);
}

// ---- RMT CKV, stock :340-460 ----

static esp_err_t ckv_setup(void)
{
    PERIPH_RCC_ATOMIC() {
        rmt_ll_enable_bus_clock(0, true);
        rmt_ll_reset_register(0);
    }
    rmt_ll_mem_force_power_on(&RMT);
    rmt_ll_enable_mem_access_nonfifo(&RMT, true);
    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src((soc_module_clk_t)RMT_CLK_SRC_DEFAULT, true), TAG, "rmt clk");
    PERIPH_RCC_ATOMIC() {
        rmt_ll_set_group_clock_src(&RMT, CKV_CHAN, RMT_CLK_SRC_DEFAULT, 1, 1, 0);
        rmt_ll_enable_group_clock(&RMT, true);
    }
    uint32_t src_hz = 0;
    ESP_RETURN_ON_ERROR(esp_clk_tree_src_get_freq_hz((soc_module_clk_t)RMT_CLK_SRC_DEFAULT,
                                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz), TAG, "rmt hz");
    const uint32_t div = (src_hz + 5000000u) / 10000000u;      // stock :400
    ESP_RETURN_ON_FALSE(div >= 1 && div <= 255 && src_hz / div == CKV_TICK_HZ, ESP_ERR_INVALID_STATE, TAG,
                        "rmt %u Hz / %u != 10 MHz", (unsigned)src_hz, (unsigned)div);
    rmt_ll_tx_set_channel_clock_div(&RMT, CKV_CHAN, div);
    rmt_ll_tx_set_mem_blocks(&RMT, CKV_CHAN, 2);                        // stock :410
    rmt_ll_tx_fix_idle_level(&RMT, CKV_CHAN, 0, true);                  // stock :411 idle low
    rmt_ll_tx_enable_carrier_modulation(&RMT, CKV_CHAN, false);         // stock :412
    rmt_ll_tx_enable_loop(&RMT, CKV_CHAN, true);                        // stock :413
    // stock :360-375: one symbol (high ckv_high, low period - high), then an end word
    volatile rmt_symbol_word_t *m = &RMTMEM[CKV_CHAN * SOC_RMT_MEM_WORDS_PER_CHANNEL];
    rmt_symbol_word_t sym = {.duration0 = s.t.ckv_high_ticks, .level0 = 1,
                             .duration1 = s.t.ckv_low_ticks, .level1 = 0};
    m[0].val = sym.val;
    m[1].val = 0;
    return ESP_OK;
}

// stock :332-345
static IRAM_ATTR void ckv_start(uint32_t cycles)
{
    rmt_ll_tx_stop(&RMT, CKV_CHAN);
    rmt_ll_tx_enable_loop_count(&RMT, CKV_CHAN, true);
    rmt_ll_tx_enable_loop_autostop(&RMT, CKV_CHAN, true);
    rmt_ll_tx_set_loop_count(&RMT, CKV_CHAN, cycles);
    rmt_ll_tx_reset_pointer(&RMT, CKV_CHAN);
    rmt_ll_tx_start(&RMT, CKV_CHAN);
}

// ---- frame ----

// stock :1319-1331, in the same critical section and the same order:
// SPV low, CKV loop start, one CKV period (integer us), SPV high, the CKV high
// time (integer us), interrupts on, LCD start
static IRAM_ATTR void frame_start_locked(void)
{
    const bool pins = s.cfg.route_pins;     // internal check: SPV stays where it is
    if (pins) {
        gpio_ll_set_level(&GPIO, s.cfg.spv, 0);
    }
    ckv_start(s.rows + CKV_EXTRA_LINES);
    esp_rom_delay_us(s.ckv_period_us);
    if (pins) {
        gpio_ll_set_level(&GPIO, s.cfg.spv, 1);
    }
    esp_rom_delay_us(s.ckv_high_us);
    lcd_ll_clear_interrupt_status(s.dev, lcd_ll_get_interrupt_status(s.dev));
    int_enable(true);
    lcd_ll_start(s.dev);
}

static esp_err_t scan_start(void *ctx, const uint8_t *buf)
{
    (void)ctx;
    const int idx = buf == s.buf[0] ? 0 : buf == s.buf[1] ? 1 : -1;
    if (!s.ready || idx < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const int64_t t_entry = esp_timer_get_time();
    if (s.cfg.route_pins) {
        if (!s.routed) {    // once per refresh, as stock keeps them routed (park unroutes)
            route(true);
            s.routed = true;
        }
        if (s.cfg.drive_oe) {
            oe(true);   // stock: XOE high after power-on (epd_board...c:952-956), MODE high for the refresh (render_lcd.c:199)
        }
    }
    // stock :1255-1268, 1296
    lcd_ll_set_horizontal_timing(s.dev, s.le_high_clk, s.front_porch_clk, s.t.line_bytes, s.t.h_front);
    lcd_ll_set_vertical_timing(s.dev, 1, 0, s.rows, 1);
    lcd_ll_set_hsync_position(s.dev, 1);
    gdma_reset(s.dma);
    lcd_ll_stop(s.dev);
    lcd_ll_clear_interrupt_status(s.dev, lcd_ll_get_interrupt_status(s.dev));
    lcd_ll_fifo_reset(s.dev);
    lcd_ll_enable_auto_next_frame(s.dev, false);
    s.eofs = 0;
    s.underrun_seen = 0;
    xSemaphoreTake(s.done, 0);
    gdma_start(s.dma, gdma_link_get_head_addr(s.link[idx]));
    portENTER_CRITICAL(&s.mux);
    s.t_rmt_start = esp_timer_get_time();
    frame_start_locked();
    s.t_lcd_start = esp_timer_get_time();
    portEXIT_CRITICAL(&s.mux);
    s.st.last_start_us = (uint32_t)(s.t_lcd_start - t_entry);
    return ESP_OK;
}

static void frame_stop(void)
{
    int_enable(false);
    lcd_ll_stop(s.dev);
    gdma_stop(s.dma);
    rmt_ll_tx_stop(&RMT, CKV_CHAN);
}

static esp_err_t scan_wait(void *ctx, uint32_t timeout_us)
{
    (void)ctx;
    const TickType_t ticks = (TickType_t)(((uint64_t)timeout_us * configTICK_RATE_HZ + 999999u) / 1000000u) + 1;
    esp_err_t err = xSemaphoreTake(s.done, ticks) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
    if (err == ESP_OK) {
        // the CKV loop runs (rows + 5) periods from before the LCD start: let it end
        const int64_t limit = esp_timer_get_time() + 4 * (int64_t)s.t.line_ns / 1000 + 100;
        while (!(rmt_ll_tx_get_interrupt_status_raw(&RMT, CKV_CHAN) & RMT_LL_EVENT_TX_LOOP_END(CKV_CHAN)) &&
               esp_timer_get_time() < limit) {
        }
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_MASK(CKV_CHAN));
    } else {
        s.st.timeouts++;
        oe(false);
    }
    frame_stop();
    if (err != ESP_OK && s.cfg.route_pins && s.routed) {
        route(false);   // a failed frame ends the refresh: pads back to their idle GPIO levels
        s.routed = false;
    }
    if (s.underrun_seen) {
        s.st.underruns++;
        oe(false);
        err = ESP_ERR_INVALID_STATE;
    }
    s.st.last_dma_eofs = s.eofs;
    if (err == ESP_OK) {
        s.st.frames++;
        s.st.last_frame_us = (uint32_t)(s.t_vsync - s.t_lcd_start);
        s.st.last_eof_us = (uint32_t)(s.t_eof - s.t_lcd_start);
        // nominal CKV end (rows + 5 periods from the RMT start) minus the last data fetch
        const int64_t ckv_end = s.t_rmt_start + (int64_t)(s.rows + CKV_EXTRA_LINES) * s.t.line_ns / 1000;
        s.st.last_ckv_margin_us = (int32_t)(ckv_end - s.t_eof);
        s.st.last_lead_ns = (uint32_t)((s.t_lcd_start - s.t_rmt_start) * 1000);
        s.st.last_tail_us = (uint32_t)(esp_timer_get_time() - s.t_vsync);
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
    frame_stop();
    if (s.routed) {
        route(false);
        s.routed = false;
    }
}

// ---- setup ----

// stock :870-920 (init_lcd_peripheral) and :1080-1110 (set_pixel_clock)
static esp_err_t lcd_setup(const epd_par_panel_t *p)
{
    PERIPH_RCC_ACQUIRE_ATOMIC(soc_lcd_rgb_signals[0].module, ref_count) {
        if (ref_count == 0) {
            lcd_ll_enable_bus_clock(0, true);
            lcd_ll_reset_register(0);
        }
    }
    s.dev = LCD_LL_GET_HW(0);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_clock(s.dev, true);
    }
    lcd_ll_mem_set_low_power_mode(s.dev, LCD_LL_MEM_LP_MODE_SHUT_DOWN);
    lcd_ll_mem_power_by_pmu(s.dev);
    lcd_ll_enable_trans_buffer(s.dev, true);
    ESP_RETURN_ON_ERROR(esp_clk_tree_enable_src((soc_module_clk_t)LCD_CLK_SRC_DEFAULT, true), TAG, "lcd clk");
    PERIPH_RCC_ATOMIC() {
        lcd_ll_select_clk_src(0, LCD_CLK_SRC_DEFAULT);
    }
    ESP_RETURN_ON_ERROR(esp_intr_alloc_intrstatus(soc_lcd_rgb_signals[0].irq_id,
                                                  ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED |
                                                      ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED,
                                                  (uint32_t)lcd_ll_get_interrupt_status_reg(s.dev),
                                                  LCD_EVENTS | LCD_LL_EVENT_UNDERRUN, lcd_isr, NULL, &s.intr),
                        TAG, "intr");
    lcd_ll_fifo_reset(s.dev);
    lcd_ll_reset(s.dev);
    lcd_ll_enable_trans_buffer(s.dev, true);
    lcd_ll_set_clock_idle_level(s.dev, false);
    lcd_ll_set_pixel_clock_edge(s.dev, false);
    lcd_ll_enable_rgb_mode(s.dev, true);
    lcd_ll_set_dma_read_stride(s.dev, 8);
    lcd_ll_set_data_wire_width(s.dev, 8);
    lcd_ll_set_phase_cycles(s.dev, 0, 0, 1);
    lcd_ll_enable_output_hsync_in_porch_region(s.dev, false);
    lcd_ll_reverse_wire_bit_order(s.dev, false);
    lcd_ll_enable_output_always_on(s.dev, true);
    lcd_ll_set_idle_level(s.dev, false, true, true);
    lcd_ll_set_blank_cycles(s.dev, 1, 1);
    lcd_ll_enable_auto_next_frame(s.dev, false);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_enable_interrupt(s.dev, LCD_EVENTS | LCD_LL_EVENT_UNDERRUN, false);
    }
    esp_intr_enable(s.intr);
    // Pixel clock (stock :1080-1110): PLL160 source, lcd_clk = 160 MHz / 4 = 40,
    // PCLK = lcd_clk / 2 = 20 MHz, the result of lcd_hal_cal_pclk_freq (mo = 2);
    // then the LCD_CAM-local clock fields stock also writes (clk_en, CLK160, div 1)
    uint32_t src_hz = 0;
    ESP_RETURN_ON_ERROR(esp_clk_tree_src_get_freq_hz((soc_module_clk_t)LCD_CLK_SRC_DEFAULT,
                                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &src_hz), TAG, "freq");
    const uint32_t div = src_hz / (p->pclk_hz * 2u);
    ESP_RETURN_ON_FALSE(div >= 2 && div * p->pclk_hz * 2u == src_hz, ESP_ERR_INVALID_ARG, TAG,
                        "pclk %u Hz from %u Hz", (unsigned)p->pclk_hz, (unsigned)src_hz);
    PERIPH_RCC_ATOMIC() {
        lcd_ll_set_group_clock_coeff(0, (int)div, 0, 0);
    }
    lcd_ll_set_pixel_clock_prescale(s.dev, 2);
    s.dev->lcd_clock.clk_en = 1;
    s.dev->lcd_clock.lcd_clk_sel = 2;
    s.dev->lcd_clock.lcd_clkm_div_num = 1;
    s.dev->lcd_clock.lcd_clkm_div_a = 0;
    s.dev->lcd_clock.lcd_clkm_div_b = 0;
    return ESP_OK;
}

static esp_err_t dma_setup(void)
{
    const gdma_channel_alloc_config_t cc = {.intr_priority = 3};    // stock :586
    ESP_RETURN_ON_ERROR(gdma_new_axi_channel(&cc, &s.dma, NULL), TAG, "dma channel");
    gdma_connect(s.dma, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    const gdma_strategy_config_t strat = {.eof_till_data_popped = false};
    gdma_apply_strategy(s.dma, &strat);
    // Not stock: stock feeds 8-line bounce buffers in internal RAM from the DMA
    // EOF interrupt; we hand the LCD the whole phase buffer (PSRAM). The pins
    // see the same bytes; an underrun ends the frame with an error.
    const gdma_transfer_config_t tc = {.max_data_burst_size = DMA_BURST, .access_ext_mem = true};
    ESP_RETURN_ON_ERROR(gdma_config_transfer(s.dma, &tc), TAG, "dma transfer");
    gdma_channel_alignment_info_t al;
    gdma_get_channel_alignment_constraints(s.dma, &al);
    for (int i = 0; i < 2; i++) {
        const gdma_link_list_config_t lc = {.num_items = s.bytes / DMA_NODE_BYTES + 2, .item_alignment = 8};
        ESP_RETURN_ON_ERROR(gdma_new_link_list(&lc, &s.link[i]), TAG, "dma link");
        const gdma_buffer_mount_config_t mc = {
            .buffer = s.buf[i], .buffer_alignment = al.ext_enc_mem_alignment ? al.ext_enc_mem_alignment : 64,
            .length = s.bytes, .flags = {.mark_eof = 1, .mark_final = GDMA_FINAL_LINK_TO_NULL},
        };
        ESP_RETURN_ON_ERROR(gdma_link_mount_buffers(s.link[i], 0, &mc, 1, NULL), TAG, "dma mount %d", i);
    }
    gdma_tx_event_callbacks_t cbs = {.on_trans_eof = on_dma_eof};
    return gdma_register_tx_event_callbacks(s.dma, &cbs, NULL);
}

esp_err_t epd_s31_bus_new(const epd_s31_bus_cfg_t *cfg, const epd_par_panel_t *panel, const epd_par_timing_t *t,
                          uint8_t *const bufs[2], epd_par_bus_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && panel && t && bufs && bufs[0] && bufs[1] && out, ESP_ERR_INVALID_ARG, TAG, "args");
    ESP_RETURN_ON_FALSE(!s.ready, ESP_ERR_INVALID_STATE, TAG, "already created");
    ESP_RETURN_ON_FALSE(panel->ckv_tick_hz == CKV_TICK_HZ, ESP_ERR_INVALID_ARG, TAG, "ckv tick");
    const soc_lcd_rgb_iomux_desc_t *mux = &soc_lcd_rgb_iomux_descs[0];
    for (int i = 0; i < 8; i++) {   // stock :665-670: data only on the LCD_DATA IOMUX pads
        ESP_RETURN_ON_FALSE(mux->data_pins[i].gpio_num == cfg->data0 + i, ESP_ERR_INVALID_ARG, TAG, "D%d pad", i);
    }
    s.cfg = *cfg;
    s.le_high_clk = panel->le_high_clk;
    s.front_porch_clk = panel->front_porch_clk;
    s.t = *t;
    s.rows = panel->rows;
    s.buf[0] = bufs[0];
    s.buf[1] = bufs[1];
    s.bytes = (size_t)t->line_bytes * panel->rows;
    // stock :1137-1150: integer microseconds of the CKV period and high time
    s.ckv_period_us = (t->ckv_period_ticks + CKV_TICK_HZ / 1000000u - 1) / (CKV_TICK_HZ / 1000000u);
    s.ckv_high_us = t->ckv_high_ticks / (CKV_TICK_HZ / 1000000u);
    s.done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s.done, ESP_ERR_NO_MEM, TAG, "sem");
    ESP_RETURN_ON_ERROR(lcd_setup(panel), TAG, "lcd");
    ESP_RETURN_ON_ERROR(dma_setup(), TAG, "dma");
    ESP_RETURN_ON_ERROR(ckv_setup(), TAG, "ckv");
    s.ready = true;
    *out = (epd_par_bus_t){.ctx = NULL, .scan_start = scan_start, .scan_wait = scan_wait, .park = park};
    ESP_LOGI(TAG, "ready: %s, %u rows x %u bytes, line %u+%u+%u+%u clk, ckv %u/%u ticks, start %u+%u us",
             cfg->route_pins ? "pins" : "internal", (unsigned)panel->rows, (unsigned)t->line_bytes,
             (unsigned)panel->le_high_clk, (unsigned)panel->front_porch_clk, (unsigned)t->line_bytes,
             (unsigned)t->h_front, (unsigned)t->ckv_high_ticks, (unsigned)t->ckv_low_ticks,
             (unsigned)s.ckv_period_us, (unsigned)s.ckv_high_us);
    return ESP_OK;
}

void epd_s31_bus_set_pins(bool route_pins, bool drive_oe)
{
    if (s.ready) {
        park(NULL);
    }
    s.cfg.route_pins = route_pins;
    s.cfg.drive_oe = drive_oe;
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
    return esp_cache_msync(p, (size_t)rows * s.t.line_bytes,
                           ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}
