#include <string.h>
#include "check.h"
#include "epd_bus_rec.h"

static int fired;
static void on_idle(void *ctx) { fired += *(int *)ctx; }

static void test_cmd_then_data(void)
{
    epd_bus_rec_t r;
    epd_bus_rec_init(&r);
    epd_bus_t *b = &r.bus;
    const uint8_t d[3] = {0xA1, 0xB2, 0xC3};
    CHECK(b->cmd(b, 0x12, NULL, 0) == ESP_OK);
    CHECK(b->data(b, d, sizeof d) == ESP_OK);
    CHECK(r.n == 2);
    CHECK(r.log[0].type == EPD_REC_CMD && r.log[0].cmd == 0x12 && r.log[0].len == 0);
    CHECK(r.log[1].type == EPD_REC_DATA && r.log[1].len == 3);
    CHECK(r.log[1].bytes && memcmp(r.log[1].bytes, d, 3) == 0);
    epd_bus_rec_free(&r);
}

static void test_cmd_with_data_is_one_entry(void)
{
    epd_bus_rec_t r;
    epd_bus_rec_init(&r);
    epd_bus_t *b = &r.bus;
    uint8_t d[2] = {0x80, 0x01};
    CHECK(b->cmd(b, 0x18, d, 2) == ESP_OK);
    d[0] = 0;   // the fake keeps its own copy
    CHECK(r.n == 1 && r.log[0].len == 2 && r.log[0].bytes[0] == 0x80 && r.log[0].bytes[1] == 0x01);
    CHECK(epd_bus_rec_find_cmd(&r, 0x18, 0) == 0);
    CHECK(epd_bus_rec_find_cmd(&r, 0x18, 1) == -1);
    CHECK(epd_bus_rec_find_cmd(&r, 0x22, 0) == -1);
    epd_bus_rec_free(&r);
}

static void test_reset_delay_and_busy_irq(void)
{
    epd_bus_rec_t r;
    epd_bus_rec_init(&r);
    epd_bus_t *b = &r.bus;
    int one = 1;
    fired = 0;
    CHECK(b->arm_busy_irq(b, on_idle, &one) == ESP_ERR_INVALID_STATE);   // before reset
    b->reset(b);
    b->delay_ms(b, 7);
    CHECK(r.log[0].type == EPD_REC_RESET);
    CHECK(r.log[1].type == EPD_REC_DELAY && r.log[1].ms == 7);

    // armed while busy: nothing until the test fires the interrupt, then exactly once
    epd_bus_rec_set_busy(&r, true);
    CHECK(b->busy(b));
    CHECK(b->arm_busy_irq(b, on_idle, &one) == ESP_OK);
    CHECK(r.log[r.n - 1].type == EPD_REC_ARM);
    CHECK(fired == 0);
    CHECK(epd_bus_rec_fire_busy(&r));
    CHECK(fired == 1 && !b->busy(b));
    CHECK(!epd_bus_rec_fire_busy(&r));
    CHECK(fired == 1);

    // armed while already idle: the callback runs at once
    CHECK(b->arm_busy_irq(b, on_idle, &one) == ESP_OK);
    CHECK(fired == 2);
    CHECK(!epd_bus_rec_fire_busy(&r));

    epd_bus_rec_clear(&r);
    CHECK(r.n == 0);
    epd_bus_rec_free(&r);
}

int main(void)
{
    test_cmd_then_data();
    test_cmd_with_data_is_one_entry();
    test_reset_delay_and_busy_irq();
    TEST_DONE("bus_rec");
}
