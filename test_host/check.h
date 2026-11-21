#pragma once
#include <stdio.h>

static int failures;
#define CHECK(cond) do { \
        if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } \
    } while (0)
#define TEST_DONE(name) do { \
        printf("%s %s (%d failure(s))\n", failures ? "FAILED" : "PASS", name, failures); \
        return failures ? 1 : 0; \
    } while (0)
