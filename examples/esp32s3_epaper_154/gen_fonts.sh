#!/usr/bin/env bash
# Regenerate the hinted Vietnamese fonts used by this example.
# Requires: pip install -r ../../tools/font_conv/requirements.txt
set -euo pipefail
cd "$(dirname "$0")"

CONV="${PYTHON:-python3} ../../tools/font_conv/epd_font_conv.py"
TTF="fonts/BeVietnamPro-Medium.ttf"

for size in 12 16; do
    # 1 bpp, FreeType mono hinting: B/W mode
    $CONV --font "$TTF" --size "$size" --bpp 1 --preset vietnamese \
        --name "font_vn_${size}" --output "main/font_vn_${size}.c"
    # 2 bpp, light hinting: 4-gray mode (levels match the panel's 4 grays)
    $CONV --font "$TTF" --size "$size" --bpp 2 --preset vietnamese \
        --name "font_vn_${size}_aa" --output "main/font_vn_${size}_aa.c"
done
