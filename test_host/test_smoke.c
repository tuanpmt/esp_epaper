#include "check.h"
#include "epd_types.h"

int main(void)
{
    epd_rect_t r = {0, 0, 8, 8};
    CHECK(r.w == 8);
    CHECK(EPD_M_FULL_CLEAN > EPD_M_FAST);
    // Merge order: a screen change without a flash keeps the dirty list (below GRAY)
    CHECK(EPD_M_FAST < EPD_M_PARTIAL && EPD_M_PARTIAL < EPD_M_TRANSITION);
    CHECK(EPD_M_TRANSITION < EPD_M_GRAY && EPD_M_GRAY < EPD_M_FULL_CLEAN);
    TEST_DONE("smoke");
}
