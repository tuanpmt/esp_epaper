// Generated E0470 waveform tables: shape and invariants.
#include "check.h"
#include "epd_wf.h"

static const epd_wf_phases_t *w(epd_wf_kind_t k)
{
    return &EPD_WF_E0470.ranges[0].w[k];
}

static void test_wf_phase_counts(void)
{
    CHECK(EPD_WF_E0470.n == 1);
    CHECK(EPD_WF_E0470.ranges[0].t_min == 0 && EPD_WF_E0470.ranges[0].t_max == 50);
    CHECK(w(EPD_WF_DU)->phases == 20);
    CHECK(w(EPD_WF_DU_FAST)->phases == 14);
    CHECK(w(EPD_WF_DU_SHORT)->phases == 8);
    CHECK(w(EPD_WF_GL16)->phases == 48);
    CHECK(w(EPD_WF_GC16)->phases == 48);
    CHECK(w(EPD_WF_GL16_TRIM)->phases == 37);
    CHECK(w(EPD_WF_GC16_TRIM)->phases == 36);
}

static void test_wf_codes_in_range(void)
{
    for (int k = 0; k < EPD_WF_COUNT; k++) {
        CHECK(w(k)->lut != NULL);
        for (int f = 0; f < w(k)->phases; f++) {
            for (int to = 0; to < 16; to++) {
                for (int from = 0; from < 16; from++) {
                    CHECK(epd_wf_action(w(k), f, from, to) <= 2);
                }
            }
        }
    }
}

// DU tables only drive towards the end point of the target (0 or 15)
static void test_wf_du_targets_endpoints(void)
{
    const epd_wf_kind_t du[] = {EPD_WF_DU, EPD_WF_DU_FAST, EPD_WF_DU_SHORT};
    for (int i = 0; i < 3; i++) {
        int towards = 0;
        for (int f = 0; f < w(du[i])->phases; f++) {
            for (int to = 0; to < 16; to++) {
                for (int from = 0; from < 16; from++) {
                    const int a = epd_wf_action(w(du[i]), f, from, to);
                    CHECK(a == 0 || a == (to < 8 ? 1 : 2));
                    towards += a != 0;
                }
            }
        }
        CHECK(towards > 0);
    }
    // a black -> white and white -> black transition is driven in every DU table
    for (int i = 0; i < 3; i++) {
        int b2w = 0, w2b = 0;
        for (int f = 0; f < w(du[i])->phases; f++) {
            b2w += epd_wf_action(w(du[i]), f, 0, 15) == 2;
            w2b += epd_wf_action(w(du[i]), f, 15, 0) == 1;
        }
        CHECK(b2w > 0 && w2b > 0);
    }
}

static void test_wf_du_black_reinforce(void)
{
    for (int f = 16; f < 20; f++) {
        CHECK(epd_wf_action(w(EPD_WF_DU), f, 0, 0) == 1);
    }
    for (int f = 4; f < 8; f++) {
        CHECK(epd_wf_action(w(EPD_WF_DU_SHORT), f, 0, 0) == 1);
    }
    CHECK(epd_wf_action(w(EPD_WF_DU_SHORT), 0, 0, 0) == 0);
    CHECK(epd_wf_action(w(EPD_WF_DU_SHORT), 0, 15, 15) == 0);
}

static void test_wf_trim_holds_tail(void)
{
    const epd_wf_kind_t tr[] = {EPD_WF_GL16_TRIM, EPD_WF_GC16_TRIM};
    for (int i = 0; i < 2; i++) {
        const int n = w(tr[i])->phases;
        for (int f = n - 3; f < n; f++) {
            for (int to = 0; to < 16; to++) {
                for (int from = 0; from < 16; from++) {
                    CHECK(epd_wf_action(w(tr[i]), f, from, to) == 0);
                }
            }
        }
    }
}

static void test_wf_gl16_white_tick(void)
{
    int n = 0;
    for (int f = 0; f < 48; f++) {
        n += epd_wf_action(w(EPD_WF_GL16), f, 15, 15) == 2;
    }
    CHECK(n >= 1);
}

int main(void)
{
    test_wf_phase_counts();
    test_wf_codes_in_range();
    test_wf_du_targets_endpoints();
    test_wf_du_black_reinforce();
    test_wf_trim_holds_tail();
    test_wf_gl16_white_tick();
    TEST_DONE("wf_data");
}
