#!/usr/bin/env python3
"""
epd_font_conv - Convert TTF/OTF fonts to LVGL 9 C fonts using FreeType hinting.

lv_font_conv rasterizes outlines without any hinting, which produces uneven
stems and broken diacritics at small sizes, especially at 1 bpp on e-paper.
This tool renders every glyph with FreeType (native TrueType hinting or the
auto-hinter), so stems snap to the pixel grid, then writes a C file in the
standard LVGL "fmt_txt" format (drop-in replacement for lv_font_conv output).

Examples:
    # 1 bpp hinted Vietnamese font for B/W e-paper
    python epd_font_conv.py --font BeVietnamPro-Medium.ttf --size 16 --bpp 1 \
        --preset vietnamese --name font_vn_16 --output font_vn_16.c --preview vn16.png

    # 2 bpp anti-aliased font for 4-gray e-paper mode
    python epd_font_conv.py --font BeVietnamPro-Medium.ttf --size 16 --bpp 2 \
        --preset vietnamese --name font_vn_16_aa --output font_vn_16_aa.c
"""

import argparse
import ctypes
import os
import re
import sys

try:
    import freetype
except ImportError:
    sys.exit("freetype-py is required: pip install -r requirements.txt")

# Every precomposed letter used in Vietnamese (lower and upper case)
VIETNAMESE_CHARS = (
    "àáảãạăằắẳẵặâầấẩẫậèéẻẽẹêềếểễệìíỉĩịòóỏõọôồốổỗộơờớởỡợùúủũụưừứửữựỳýỷỹỵđ"
    "ÀÁẢÃẠĂẰẮẲẴẶÂẦẤẨẪẬÈÉẺẼẸÊỀẾỂỄỆÌÍỈĨỊÒÓỎÕỌÔỒỐỔỖỘƠỜỚỞỠỢÙÚỦŨỤƯỪỨỬỮỰỲÝỶỸỴĐ"
)

PRESETS = {
    "ascii": set(range(0x20, 0x7F)),
    "vietnamese": set(range(0x20, 0x7F)) | {ord(c) for c in VIETNAMESE_CHARS},
}

HINTING_MODES = ("mono", "light", "normal", "auto", "auto-light", "none")


def parse_ranges(values):
    codes = set()
    for value in values or []:
        for part in value.split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                lo, hi = part.split("-", 1)
                lo, hi = int(lo, 0), int(hi, 0)
                if hi < lo:
                    raise ValueError(f"Invalid range: {part}")
                codes.update(range(lo, hi + 1))
            else:
                codes.add(int(part, 0))
    return codes


def load_flags_for(hinting, bpp):
    """Map a hinting mode to FreeType load flags."""
    f = freetype
    if hinting == "none":
        return f.FT_LOAD_NO_HINTING
    if hinting == "mono":
        return f.FT_LOAD_TARGET_MONO
    if hinting == "light":
        return f.FT_LOAD_TARGET_LIGHT
    if hinting == "normal":
        return f.FT_LOAD_TARGET_NORMAL
    if hinting == "auto":
        target = f.FT_LOAD_TARGET_MONO if bpp == 1 else f.FT_LOAD_TARGET_NORMAL
        return f.FT_LOAD_FORCE_AUTOHINT | target
    if hinting == "auto-light":
        return f.FT_LOAD_FORCE_AUTOHINT | f.FT_LOAD_TARGET_LIGHT
    raise ValueError(hinting)


class Glyph:
    __slots__ = ("code", "adv_w", "box_w", "box_h", "ofs_x", "ofs_y", "pixels")

    def __init__(self, code, adv_w, box_w, box_h, ofs_x, ofs_y, pixels):
        self.code = code
        self.adv_w = adv_w      # advance in 1/16 px
        self.box_w = box_w
        self.box_h = box_h
        self.ofs_x = ofs_x
        self.ofs_y = ofs_y      # bbox bottom relative to baseline (positive = up)
        self.pixels = pixels    # rows of quantized values (0 .. 2^bpp - 1)


def render_glyph(face, code, args):
    """Render one glyph and return a Glyph with quantized pixels."""
    flags = load_flags_for(args.hinting, args.bpp)
    face.load_char(chr(code), flags)
    slot = face.glyph

    if args.embolden:
        strength = int(round(args.embolden * 64))
        freetype.FT_Outline_Embolden(ctypes.byref(slot.outline._FT_Outline), strength)

    max_val = (1 << args.bpp) - 1
    use_mono = args.bpp == 1 and args.threshold is None
    slot.render(freetype.FT_RENDER_MODE_MONO if use_mono else freetype.FT_RENDER_MODE_NORMAL)
    bmp = slot.bitmap

    # Advance: hinted fonts give integer pixel advances; keep 1/16 px precision otherwise
    adv_px = slot.advance.x / 64.0
    if args.hinting != "none" or args.round_advance:
        adv_px = round(adv_px)
    adv_px += args.letter_spacing
    adv_w = max(0, int(round(adv_px * 16)))

    rows, width, pitch = bmp.rows, bmp.width, bmp.pitch
    buf = bmp.buffer
    pixels = []
    for y in range(rows):
        row = []
        base = y * abs(pitch)
        for x in range(width):
            if use_mono:
                v = (buf[base + (x >> 3)] >> (7 - (x & 7))) & 1
            else:
                g = buf[base + x]
                if args.bpp == 1:
                    v = 1 if g >= args.threshold else 0
                else:
                    v = (g * max_val + 127) // 255
            row.append(v)
        pixels.append(row)

    left, top = slot.bitmap_left, slot.bitmap_top

    # Trim empty rows/columns so the bitmap stays tight
    while pixels and not any(pixels[0]):
        pixels.pop(0)
        top -= 1
    while pixels and not any(pixels[-1]):
        pixels.pop()
    if pixels:
        while pixels[0] and not any(r[0] for r in pixels):
            for r in pixels:
                r.pop(0)
            left += 1
        while pixels[0] and not any(r[-1] for r in pixels):
            for r in pixels:
                r.pop()

    box_h = len(pixels)
    box_w = len(pixels[0]) if box_h else 0
    if box_w == 0 or box_h == 0:
        return Glyph(code, adv_w, 0, 0, 0, 0, [])

    ofs_y = top - box_h
    if box_w > 255 or box_h > 255 or not (-128 <= left <= 127) or not (-128 <= ofs_y <= 127):
        raise ValueError(f"Glyph U+{code:04X} is too large for LVGL fmt_txt (size too big?)")
    return Glyph(code, adv_w, box_w, box_h, left, ofs_y, pixels)


def pack_bitmap(glyph, bpp):
    """Pack pixels as a continuous MSB-first bit stream (LVGL stride = 0)."""
    out = bytearray()
    acc, nbits = 0, 0
    for row in glyph.pixels:
        for v in row:
            acc = (acc << bpp) | v
            nbits += bpp
            if nbits == 8:
                out.append(acc)
                acc, nbits = 0, 0
    if nbits:
        out.append(acc << (8 - nbits))
    return bytes(out)


def build_cmaps(codes):
    """Split sorted code points into contiguous FORMAT0_TINY ranges plus one sparse table."""
    runs, cur = [], [codes[0]]
    for c in codes[1:]:
        if c == cur[-1] + 1:
            cur.append(c)
        else:
            runs.append(cur)
            cur = [c]
    runs.append(cur)

    cmaps, sparse = [], []
    gid = 1
    for run in runs:
        if len(run) >= 4:
            cmaps.append({"type": "FORMAT0_TINY", "start": run[0], "length": len(run), "gid": gid, "list": None})
            gid += len(run)
        else:
            sparse.extend(run)
    order = [c for r in runs if len(r) >= 4 for c in r]
    if sparse:
        start = sparse[0]
        cmaps.append({"type": "SPARSE_TINY", "start": start, "length": sparse[-1] - start + 1,
                      "gid": gid, "list": [c - start for c in sparse]})
        order.extend(sparse)
    return cmaps, order


def c_char_comment(code):
    ch = chr(code)
    if ch in ('\\', '"') or code < 0x20:
        return f"U+{code:04X}"
    return f"U+{code:04X} \"{ch}\""


def write_c_file(path, name, args, glyphs, line_height, base_line, ul_pos, ul_thick, cmdline):
    guard = re.sub(r"[^A-Z0-9]", "_", name.upper())
    codes = sorted(glyphs)
    cmaps, order = build_cmaps(codes)

    lines = []
    w = lines.append
    w("/*******************************************************************************")
    w(f" * Size: {args.size} px")
    w(f" * Bpp: {args.bpp}")
    w(f" * Hinting: {args.hinting}")
    w(" * Generated by tools/font_conv/epd_font_conv.py (FreeType hinted rasterizer)")
    w(f" * Cmd: {cmdline}")
    w(" ******************************************************************************/")
    w("")
    w("#ifdef LV_LVGL_H_INCLUDE_SIMPLE")
    w("#include \"lvgl.h\"")
    w("#else")
    w("#include \"lvgl.h\"")
    w("#endif")
    w("")
    w(f"#ifndef {guard}")
    w(f"#define {guard} 1")
    w("#endif")
    w("")
    w(f"#if {guard}")
    w("")
    w("/*-----------------")
    w(" *    BITMAPS")
    w(" *----------------*/")
    w("")
    w("static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {")

    dsc_rows = ["    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */"]
    index = 0
    for code in order:
        g = glyphs[code]
        data = pack_bitmap(g, args.bpp) if g.box_w else b""
        w(f"    /* {c_char_comment(code)} */")
        if data:
            for i in range(0, len(data), 12):
                w("    " + ", ".join(f"0x{b:02x}" for b in data[i:i + 12]) + ",")
        w("")
        dsc_rows.append(
            f"    {{.bitmap_index = {index}, .adv_w = {g.adv_w}, .box_w = {g.box_w}, "
            f".box_h = {g.box_h}, .ofs_x = {g.ofs_x}, .ofs_y = {g.ofs_y}}}"
        )
        index += len(data)
    if index == 0:
        w("    0x00,")
    w("};")
    if index >= (1 << 20):
        raise ValueError("Bitmap data exceeds 1 MB (LV_FONT_FMT_TXT_LARGE needed)")

    w("")
    w("/*---------------------")
    w(" *  GLYPH DESCRIPTION")
    w(" *--------------------*/")
    w("")
    w("static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {")
    w(",\n".join(dsc_rows))
    w("};")
    w("")
    w("/*---------------------")
    w(" *  CHARACTER MAPPING")
    w(" *--------------------*/")
    w("")
    for i, cm in enumerate(cmaps):
        if cm["list"] is not None:
            w(f"static const uint16_t unicode_list_{i}[] = {{")
            items = cm["list"]
            for j in range(0, len(items), 8):
                w("    " + ", ".join(f"0x{v:x}" for v in items[j:j + 8]) + ",")
            w("};")
            w("")
    w("static const lv_font_fmt_txt_cmap_t cmaps[] = {")
    entries = []
    for i, cm in enumerate(cmaps):
        if cm["list"] is None:
            entries.append(
                f"    {{\n        .range_start = {cm['start']}, .range_length = {cm['length']}, "
                f".glyph_id_start = {cm['gid']},\n        .unicode_list = NULL, .glyph_id_ofs_list = NULL, "
                f".list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY\n    }}"
            )
        else:
            entries.append(
                f"    {{\n        .range_start = {cm['start']}, .range_length = {cm['length']}, "
                f".glyph_id_start = {cm['gid']},\n        .unicode_list = unicode_list_{i}, "
                f".glyph_id_ofs_list = NULL, .list_length = {len(cm['list'])}, "
                f".type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY\n    }}"
            )
    w(",\n".join(entries))
    w("};")
    w("")
    w("/*--------------------")
    w(" *  ALL CUSTOM DATA")
    w(" *--------------------*/")
    w("")
    w("static const lv_font_fmt_txt_dsc_t font_dsc = {")
    w("    .glyph_bitmap = glyph_bitmap,")
    w("    .glyph_dsc = glyph_dsc,")
    w("    .cmaps = cmaps,")
    w("    .kern_dsc = NULL,")
    w("    .kern_scale = 0,")
    w(f"    .cmap_num = {len(cmaps)},")
    w(f"    .bpp = {args.bpp},")
    w("    .kern_classes = 0,")
    w("    .bitmap_format = 0,")
    w("};")
    w("")
    w("/*-----------------")
    w(" *  PUBLIC FONT")
    w(" *----------------*/")
    w("")
    w(f"const lv_font_t {name} = {{")
    w("    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,")
    w("    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,")
    w(f"    .line_height = {line_height},")
    w(f"    .base_line = {base_line},")
    w("    .subpx = LV_FONT_SUBPX_NONE,")
    w(f"    .underline_position = {ul_pos},")
    w(f"    .underline_thickness = {ul_thick},")
    w("    .dsc = &font_dsc,")
    w("    .fallback = NULL,")
    w("    .user_data = NULL,")
    w("};")
    w("")
    w(f"#endif /* #if {guard} */")
    w("")

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    return index


def render_preview(path, glyphs, args, line_height, base_line, text, scale):
    """Render sample text the way LVGL would draw it on a B/W (or 4-gray) panel."""
    try:
        from PIL import Image
    except ImportError:
        print("Pillow not installed, skipping preview", file=sys.stderr)
        return

    max_val = (1 << args.bpp) - 1
    text_lines = text.split("\\n")
    width = 0
    for tl in text_lines:
        width = max(width, sum(glyphs[ord(c)].adv_w for c in tl if ord(c) in glyphs) // 16)
    pad = 4
    img_w = width + pad * 2
    img_h = line_height * len(text_lines) + pad * 2
    img = Image.new("L", (img_w, img_h), 255)
    px = img.load()

    for li, tl in enumerate(text_lines):
        pen_x = pad * 16
        line_top = pad + li * line_height
        for c in tl:
            g = glyphs.get(ord(c))
            if not g:
                continue
            gx = pen_x // 16 + g.ofs_x
            gy = line_top + (line_height - base_line) - g.box_h - g.ofs_y
            for y, row in enumerate(g.pixels):
                for x, v in enumerate(row):
                    if v and 0 <= gx + x < img_w and 0 <= gy + y < img_h:
                        px[gx + x, gy + y] = 255 - (255 * v // max_val)
            pen_x += g.adv_w

    if scale > 1:
        img = img.resize((img_w * scale, img_h * scale), Image.NEAREST)
    img.save(path)


def main():
    p = argparse.ArgumentParser(description="Convert fonts to LVGL 9 C files with FreeType hinting")
    p.add_argument("--font", required=True, help="TTF/OTF font file")
    p.add_argument("--size", type=int, required=True, help="Font size in pixels")
    p.add_argument("--bpp", type=int, choices=(1, 2, 4, 8), default=1, help="Bits per pixel (default 1)")
    p.add_argument("--name", required=True, help="C symbol name of the font, e.g. font_vn_16")
    p.add_argument("--output", required=True, help="Output .c file")
    p.add_argument("--preset", choices=sorted(PRESETS), action="append", help="Character set preset (repeatable)")
    p.add_argument("--range", action="append", help="Code point ranges, e.g. 0x20-0x7E,0xB0")
    p.add_argument("--symbols", default="", help="Extra characters to include")
    p.add_argument("--hinting", choices=HINTING_MODES, default=None,
                   help="Hinting mode (default: mono for 1 bpp, light for >1 bpp)")
    p.add_argument("--threshold", type=int, default=None,
                   help="1 bpp only: render anti-aliased and threshold at this level (0-255) "
                        "instead of FreeType mono rendering. Lower = bolder")
    p.add_argument("--embolden", type=float, default=0.0, help="Embolden outlines by N pixels (e.g. 0.5)")
    p.add_argument("--letter-spacing", type=int, default=0, help="Extra pixels added to every advance")
    p.add_argument("--round-advance", action="store_true", help="Round advances to whole pixels even without hinting")
    p.add_argument("--preview", help="Write a PNG preview of sample text")
    p.add_argument("--preview-text", default="Màn hình E-Paper\\nXin chào Việt Nam!\\nĐẾM SỐ: 0123456789",
                   help="Preview text, use \\n for new lines")
    p.add_argument("--preview-scale", type=int, default=4, help="Preview zoom factor (default 4)")
    args = p.parse_args()

    if args.hinting is None:
        args.hinting = "mono" if args.bpp == 1 else "light"
    if args.threshold is not None and args.bpp != 1:
        p.error("--threshold is only valid with --bpp 1")
    if args.hinting == "mono" and args.bpp != 1:
        print("warning: mono hinting is tuned for 1 bpp output", file=sys.stderr)

    codes = set()
    for preset in args.preset or []:
        codes |= PRESETS[preset]
    codes |= parse_ranges(args.range)
    codes |= {ord(c) for c in args.symbols}
    if not codes:
        codes = set(PRESETS["ascii"])

    face = freetype.Face(args.font)
    face.set_pixel_sizes(0, args.size)

    glyphs, missing = {}, []
    for code in sorted(codes):
        if face.get_char_index(code) == 0:
            missing.append(code)
            continue
        glyphs[code] = render_glyph(face, code, args)
    if not glyphs:
        sys.exit("No glyphs found in font for the requested characters")
    if missing:
        shown = "".join(chr(c) for c in missing[:40])
        print(f"warning: {len(missing)} characters missing from font: {shown}", file=sys.stderr)

    # Line metrics from the real glyph extents so stacked diacritics (Ấ, Ỗ) never clip
    metrics = face.size
    asc = int(round(metrics.ascender / 64.0))
    desc = int(round(metrics.descender / 64.0))
    top = max([asc] + [g.ofs_y + g.box_h for g in glyphs.values() if g.box_h])
    bottom = min([desc] + [g.ofs_y for g in glyphs.values() if g.box_h])
    line_height = top - bottom
    base_line = -bottom

    upem = face.units_per_EM or 1000
    ul_pos = int(round(face.underline_position * args.size / upem))
    ul_thick = max(1, int(round(face.underline_thickness * args.size / upem)))

    cmdline = "epd_font_conv.py " + " ".join(
        os.path.basename(a) if a == args.font else a for a in sys.argv[1:])
    size = write_c_file(args.output, args.name, args, glyphs, line_height, base_line,
                        ul_pos, ul_thick, cmdline)
    print(f"{args.output}: {len(glyphs)} glyphs, {size} bytes bitmap, "
          f"line_height={line_height}, base_line={base_line}")

    if args.preview:
        render_preview(args.preview, glyphs, args, line_height, base_line,
                       args.preview_text, args.preview_scale)
        print(f"preview: {args.preview}")


if __name__ == "__main__":
    main()
