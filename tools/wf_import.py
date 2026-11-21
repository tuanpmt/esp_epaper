#!/usr/bin/env python3
"""Generate the E0470 waveform tables of esp_epaper from the vendor firmware.

Usage: wf_import.py <stock waveforms dir> <out.c>

The stock dir is main/boards/metalio-e-ink4-plus/display/waveforms of
CloudZao/Metalio-E-INK4-Plus (MIT). Read: my_waveform_du.h, my_waveform_gc.h,
my_waveform_gl.h (vendor tables, "EPD Waveform Engineering Studio V2").
The vendor builds its derived tables at boot (my_waveform.c,
my_waveform_trim.c); this script applies the same derivations at build time:

  DU 20      source DU only targets 0/15: every target gets the row of its
             end point (to < 8 -> 0, else 15); 0->0 gets one darken pulse in
             each of the last 4 phases (black reinforcement)
  DU short 8 / fast 14
             every from != to drives straight to the end point: 7/13 darken
             phases to black, 8/14 lighten phases to white; 0->0 darken in
             phases 4..7 / 10..13
  GL16 48    vendor table plus the "white tick": 15->15 lightens once, in the
             last phase where some other level is lightened towards 15
  GC16 48    vendor table as is
  GC16/GL16 trimmed (36/37)
             per (to, from): drop leading/trailing no-ops, keep at most 11
             erase phases, cut 5 phases off the start of the saturation run
             (0 for to = 15), right-align, keep 3 trailing hold phases;
             GL16 trimmed gets the white tick afterwards

The EPDiy tables in the same directory (epdiy_*.h, LGPL) and the gray8 tables
(Apache-2.0) are not read.
"""
import os
import re
import sys

LEVELS = 16

MIT_NOTICE = """\
 * Source data: CloudZao/Metalio-E-INK4-Plus,
 * main/boards/metalio-e-ink4-plus/display/waveforms/my_waveform_{du,gc,gl}.h
 * and the derivations of my_waveform.c / my_waveform_trim.c, used under:
 *
 * MIT License
 *
 * Copyright (c) 2025 Shenzhen Xinzhi Future Technology Co., Ltd.
 * Copyright (c) 2025 Project Contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE."""


class Table:
    """actions[phase][to][from] in 0..3"""

    def __init__(self, phases):
        self.a = [[[0] * LEVELS for _ in range(LEVELS)] for _ in range(phases)]

    @property
    def phases(self):
        return len(self.a)

    def get(self, f, to, frm):
        return self.a[f][to][frm]

    def orr(self, f, to, frm, act):
        self.a[f][to][frm] |= act

    def set(self, f, to, frm, act):
        self.a[f][to][frm] = act

    def pack(self):
        out = []
        for f in range(self.phases):
            for to in range(LEVELS):
                for b in range(4):
                    v = 0
                    for k in range(4):
                        v |= (self.a[f][to][b * 4 + k] & 3) << (6 - 2 * k)
                    out.append(v)
        return out


def parse(path, name):
    text = open(path, encoding="utf-8").read()
    m = re.search(r"const\s+uint8_t\s+" + re.escape(name) + r"\s*\[(\d+)\]\[16\]\[4\]\s*=\s*\{(.*?)\};", text, re.S)
    if not m:
        sys.exit(f"{path}: table {name} not found")
    phases = int(m.group(1))
    vals = [int(x, 16) for x in re.findall(r"0x[0-9A-Fa-f]{2}", m.group(2))]
    if len(vals) > phases * LEVELS * 4 or len(vals) % (LEVELS * 4):
        sys.exit(f"{path}: {name}: {len(vals)} bytes, expected {phases * 64}")
    if len(vals) < phases * LEVELS * 4:
        # C zero-fills missing initializers (my_waveform_gc.h lists 47 of 48 phases)
        print(f"{name}: {len(vals) // 64} of {phases} phases given, rest zero as in C")
        vals += [0] * (phases * LEVELS * 4 - len(vals))
    t = Table(phases)
    i = 0
    for f in range(phases):
        for to in range(LEVELS):
            for b in range(4):
                for k in range(4):
                    t.a[f][to][b * 4 + k] = (vals[i] >> (6 - 2 * k)) & 3
                i += 1
    return t


def du_full(src):
    t = Table(src.phases)
    for to in range(LEVELS):
        end = 0 if to < 8 else 15
        for frm in range(LEVELS):
            for f in range(src.phases):
                act = src.get(f, end, frm)
                if act:
                    t.orr(f, to, frm, act)
    for f in range(src.phases - 4, src.phases):
        t.orr(f, 0, 0, 1)
    return t


def du_endpoint(phases, black_phases, reinforce):
    t = Table(phases)
    for to in range(LEVELS):
        for frm in range(LEVELS):
            if to == frm:
                continue
            end = 0 if to < 8 else 15
            act = 1 if end == 0 else 2
            n = black_phases if end == 0 else phases
            for f in range(n):
                t.orr(f, to, frm, act)
    for f in reinforce:
        t.orr(f, 0, 0, 1)
    return t


def white_tick(t):
    tick = -1
    for f in range(t.phases - 1, -1, -1):
        if any(t.get(f, 15, frm) == 2 for frm in range(15)):
            tick = f
            break
    if tick < 0:
        tick = t.phases - 3 if t.phases > 2 else 0
    t.orr(tick, 15, 15, 2)
    return t


def copy(src):
    t = Table(src.phases)
    for f in range(src.phases):
        for to in range(LEVELS):
            t.a[f][to] = list(src.a[f][to])
    return t


def trim_one(seq, to, erase_max, sat_cut, white_sat_cut):
    sat = 2 if to == 15 else 1
    erase = 1 if to == 15 else 2
    a = 0
    while a < len(seq) and seq[a] == 0:
        a += 1
    z = len(seq)
    while z > a and seq[z - 1] == 0:
        z -= 1
    if a >= z:
        return []

    def run(start, value):
        n = 0
        while start + n < z and seq[start + n] == value:
            n += 1
        return n

    e = run(a, erase)
    s = run(a + e, sat)
    keep_e = min(e, erase_max)
    want = white_sat_cut if to == 15 else sat_cut
    cut = max(0, min(want, s - 1))
    return [erase] * keep_e + [sat] * (s - cut) + list(seq[a + e + s:z])


def trim(src, erase_max=11, sat_cut=5, white_sat_cut=0, hold=3):
    seqs = {}
    longest = 0
    for to in range(LEVELS):
        for frm in range(LEVELS):
            s = trim_one([src.get(f, to, frm) for f in range(src.phases)], to, erase_max, sat_cut, white_sat_cut)
            seqs[(to, frm)] = s
            longest = max(longest, len(s))
    phases = longest + hold
    if phases > src.phases:
        sys.exit("trim: longer than the source")
    t = Table(phases)
    for (to, frm), s in seqs.items():
        start = longest - len(s)
        for i, act in enumerate(s):
            t.set(start + i, to, frm, act)
    return t


def emit(out, tables):
    lines = [
        "// GENERATED by tools/wf_import.py: do not edit.",
        "/*",
        " * E0470A14-AF (ED047TC2) waveforms for esp_epaper, one temperature range",
        " * (0..50 C, as the vendor firmware). Format: see include/epd_wf.h.",
        " *",
        MIT_NOTICE,
        " */",
        "",
        '#include "epd_wf.h"',
        "",
    ]
    for name, t in tables:
        data = t.pack()
        lines.append(f"static const uint8_t {name}[{t.phases}][16][4] = {{")
        for f in range(t.phases):
            rows = []
            for to in range(LEVELS):
                b = data[(f * LEVELS + to) * 4:(f * LEVELS + to) * 4 + 4]
                rows.append("{" + ",".join(f"0x{x:02X}" for x in b) + "}")
            lines.append("    {" + ",".join(rows) + "},")
        lines.append("};")
        lines.append("")
    kinds = [("EPD_WF_DU", "e0470_du"), ("EPD_WF_DU_FAST", "e0470_du_fast"), ("EPD_WF_DU_SHORT", "e0470_du_short"),
             ("EPD_WF_GL16", "e0470_gl16"), ("EPD_WF_GC16", "e0470_gc16"), ("EPD_WF_GL16_TRIM", "e0470_gl16_trim"),
             ("EPD_WF_GC16_TRIM", "e0470_gc16_trim")]
    phases = {name: t.phases for name, t in tables}
    lines.append("static const epd_wf_range_t e0470_ranges[] = {")
    lines.append("    {.t_min = 0, .t_max = 50, .w = {")
    for kind, name in kinds:
        lines.append(f"        [{kind}] = {{.phases = {phases[name]}, .lut = &{name}[0][0][0]}},")
    lines.append("    }},")
    lines.append("};")
    lines.append("")
    lines.append("const epd_wf_set_t EPD_WF_E0470 = {.n = 1, .ranges = e0470_ranges};")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    d, out = sys.argv[1], sys.argv[2]
    du = parse(os.path.join(d, "my_waveform_du.h"), "my_du_25_0_data")
    gc = parse(os.path.join(d, "my_waveform_gc.h"), "my_gc_25_0_data")
    gl = parse(os.path.join(d, "my_waveform_gl.h"), "my_gl_25_0_data")
    if (du.phases, gc.phases, gl.phases) != (20, 48, 48):
        sys.exit("unexpected phase counts in the vendor tables")
    tables = [
        ("e0470_du", du_full(du)),
        ("e0470_du_fast", du_endpoint(14, 13, range(10, 14))),
        ("e0470_du_short", du_endpoint(8, 7, range(4, 8))),
        ("e0470_gl16", white_tick(copy(gl))),
        ("e0470_gc16", copy(gc)),
        ("e0470_gl16_trim", white_tick(trim(gl))),
        ("e0470_gc16_trim", trim(gc)),
    ]
    emit(out, tables)
    for name, t in tables:
        print(f"{name}: {t.phases} phases")


if __name__ == "__main__":
    main()
