#include "check.h"
#include "epd_types.h"

int main(void)
{
    epd_rect_t r = {0, 0, 8, 8};
    CHECK(r.w == 8);
    CHECK(EPD_M_FULL_CLEAN > EPD_M_FAST);
    TEST_DONE("smoke");
}
