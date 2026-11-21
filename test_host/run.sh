#!/usr/bin/env bash
# Host unit tests for the panel-independent esp_epaper v3 modules.
# Usage: test_host/run.sh [name-filter]. Exit code != 0 when a test fails.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${EPD_TEST_OUT:-$ROOT/build_host}"
mkdir -p "$OUT"
CFLAGS=(-std=c11 -Wall -Wextra -Werror -g -fsanitize=address,undefined -D_DEFAULT_SOURCE)
INC=(-I "$ROOT/include" -I "$ROOT/src" -I "$ROOT/test_host")
STATUS=0
FILTER="${1:-}"

# run_test <name> <sources...>   (paths relative to the repo root)
run_test() {
    local name="$1"; shift
    [ -n "$FILTER" ] && [[ "$name" != *"$FILTER"* ]] && return
    local srcs=()
    for s in "$@"; do srcs+=("$ROOT/$s"); done
    if cc "${CFLAGS[@]}" "${INC[@]}" "${srcs[@]}" -o "$OUT/$name" -lm; then
        "$OUT/$name" || STATUS=1
    else
        echo "BUILD FAIL $name"; STATUS=1
    fi
}

# One line per test binary
run_test test_smoke test_host/test_smoke.c
run_test test_sched test_host/test_sched.c src/epd_sched.c
run_test test_quant src/epd_quant.c test_host/test_quant.c
run_test test_dirty src/epd_dirty.c test_host/test_dirty.c
run_test test_bus_rec test_host/epd_bus_rec.c test_host/test_bus_rec.c

exit $STATUS
