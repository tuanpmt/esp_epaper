# epd_font_conv

Converts TTF/OTF fonts into LVGL 9 C fonts (`lv_font_fmt_txt`) using FreeType hinting.
The output works anywhere an `lv_font_conv` font does (`LV_FONT_DECLARE` + `lv_obj_set_style_text_font`).

## Why

`lv_font_conv` rasterizes outlines without hinting. At small sizes and 1 bpp the stems
come out 1 px in one letter and 2 px in the next, and stacked Vietnamese diacritics
(ệ, ở, ấ) merge into the letter. FreeType hinting snaps stems and diacritics to the
pixel grid, so the result looks close to a hand-drawn bitmap font.

## Install

```bash
pip install -r requirements.txt
```

## Usage

```bash
# B/W panel: 1 bpp, FreeType mono hinting
python epd_font_conv.py --font BeVietnamPro-Medium.ttf --size 16 --bpp 1 \
    --preset vietnamese --name font_vn_16 --output font_vn_16.c --preview vn16.png

# 4-gray panel: 2 bpp anti-aliased (levels match the 4 panel grays)
python epd_font_conv.py --font BeVietnamPro-Medium.ttf --size 16 --bpp 2 \
    --preset vietnamese --name font_vn_16_aa --output font_vn_16_aa.c
```

In the firmware:

```c
LV_FONT_DECLARE(font_vn_16);
lv_obj_set_style_text_font(label, &font_vn_16, 0);
```

## Options

| Option | Description |
|--------|-------------|
| `--font` | TTF/OTF file |
| `--size` | Size in pixels |
| `--bpp` | 1, 2, 4 or 8 (default 1) |
| `--name` | C symbol name |
| `--output` | Output `.c` file |
| `--preset` | `ascii`, `vietnamese` (ASCII + all Vietnamese letters); repeatable |
| `--range` | Code point ranges, e.g. `0x20-0x7E,0xB0` |
| `--symbols` | Extra characters, e.g. `"°±€"` |
| `--hinting` | `mono` (default for 1 bpp), `light` (default for >1 bpp), `normal`, `auto`, `auto-light`, `none` |
| `--threshold N` | 1 bpp only: render anti-aliased then threshold at N (0-255); lower values give bolder text |
| `--embolden PX` | Thicken outlines by PX pixels (e.g. `0.5`) for thin fonts |
| `--letter-spacing PX` | Extra pixels added to every advance |
| `--preview FILE.png` | Render sample text to a PNG (zoomed) to check before flashing |
| `--preview-text` | Preview text, `\n` for new lines |

## Choosing a font

- Use a sans-serif with even stroke width and a large x-height. Thin serif fonts lose
  their hairlines at 1 bpp.
- Prefer Medium or SemiBold over Regular: 1 bpp rendering makes strokes look thinner.
- For Vietnamese, use 14-16 px and up for body text on ~180 dpi panels (1.54" 200x200);
  12 px is the readable minimum.
- If `mono` hinting looks wrong for a font (bad hinting instructions), try `--hinting auto`.
- Fonts tested: Be Vietnam Pro (OFL, designed with full Vietnamese support),
  available at https://github.com/google/fonts/tree/main/ofl/bevietnampro

## Notes

- Kerning is not exported (hinted fonts use whole-pixel advances).
- `line_height` is computed from the real glyph extents, so stacked diacritics on capitals
  (Ấ, Ỗ) never clip. This makes the line height larger than the nominal font size.
