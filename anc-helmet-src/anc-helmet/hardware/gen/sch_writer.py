"""Write a KiCad 7 schematic (.kicad_sch) from a Circuit.

Style: every symbol pin gets a short wire stub and a net label (the common
"label-connected" style). Parts are grouped into framed functional blocks,
laid out in columns. Unused pins get no-connect flags; every power net gets a
PWR_FLAG so ERC knows it is driven.
"""
from __future__ import annotations

import datetime
import uuid as _uuid

import kisym

GRID = 1.27
STUB = 2.54


def U():
    return str(_uuid.uuid4())


def snap(v, g=2.54):
    return round(round(v / g) * g, 4)


def q(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"') + '"'


def sym_units(sym):
    """unit number -> list of pins; unit 0 pins are common to every unit."""
    ps = kisym.pins(sym)
    units = sorted({p[6] for p in ps if p[6] != 0}) or [0]
    common = [p for p in ps if p[6] == 0]
    out = {}
    for u in units:
        out[u] = [p for p in ps if p[6] == u] + ([] if u == 0 else common)
    return out


def body_extent(sym, unit):
    """(xmin, xmax, ymin, ymax) of pins + graphics for a unit, in lib coords (y up)."""
    xs, ys = [], []
    for e in sym:
        if isinstance(e, list) and e[0] == "symbol":
            name = e[1].strip('"')
            try:
                u = int(name.rsplit("_", 2)[-2])
            except ValueError:
                u = 0
            if u not in (0, unit):
                continue
            for g in e[2:]:
                if not isinstance(g, list):
                    continue
                if g[0] == "pin":
                    at = [x for x in g if isinstance(x, list) and x[0] == "at"][0]
                    xs.append(float(at[1])); ys.append(float(at[2]))
                if g[0] in ("rectangle",):
                    for k in ("start", "end"):
                        pt = [x for x in g if isinstance(x, list) and x[0] == k][0]
                        xs.append(float(pt[1])); ys.append(float(pt[2]))
                if g[0] in ("polyline",):
                    pts = [x for x in g if isinstance(x, list) and x[0] == "pts"][0]
                    for xy in pts[1:]:
                        xs.append(float(xy[1])); ys.append(float(xy[2]))
                if g[0] == "circle":
                    c = [x for x in g if isinstance(x, list) and x[0] == "center"][0]
                    r = float([x for x in g if isinstance(x, list) and x[0] == "radius"][0][1])
                    xs += [float(c[1]) - r, float(c[1]) + r]; ys += [float(c[2]) - r, float(c[2]) + r]
    if not xs:
        return (-2.54, 2.54, -2.54, 2.54)
    return (min(xs), max(xs), min(ys), max(ys))


class SchWriter:
    def __init__(self, circ, paper="A1", rev="A", company="Open ANC Helmet"):
        self.c = circ
        self.paper = paper
        self.rev = rev
        self.company = company
        self.root = U()
        self.items = []
        self.lib = {}
        self.flag_n = 0
        self.placed_units = []   # (ref, unit, x, y)
        self.label_count = 0

    # ------------------------------------------------------------------ primitives
    def _lib(self, lib_id, sym):
        if lib_id in self.lib:
            return
        s = list(sym)
        s[1] = q(lib_id)
        self.lib[lib_id] = kisym.dump(s, 1)

    def wire(self, x1, y1, x2, y2):
        self.items.append(f'(wire (pts (xy {x1} {y1}) (xy {x2} {y2})) (stroke (width 0) (type default)) (uuid {U()}))')

    def label(self, net, x, y, ang):
        just = {0: "left bottom", 90: "left bottom", 180: "right bottom", 270: "right bottom"}[ang]
        self.items.append(f'(label {q(net)} (at {x} {y} {ang}) (fields_autoplaced) '
                          f'(effects (font (size 1.27 1.27)) (justify {just})) (uuid {U()}))')
        self.label_count += 1

    def noconn(self, x, y):
        self.items.append(f'(no_connect (at {x} {y}) (uuid {U()}))')

    def text(self, s, x, y, size=2.54, bold=True):
        b = " bold" if bold else ""
        self.items.append(f'(text {q(s)} (at {x} {y} 0) (effects (font (size {size} {size}){b}) (justify left bottom)) (uuid {U()}))')

    def rect(self, x1, y1, x2, y2):
        self.items.append(f'(rectangle (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.254) (type dash)) (fill (type none)) (uuid {U()}))')

    def symbol_inst(self, lib_id, ref, value, fp, x, y, unit, pins, props, ext=None):
        pin_s = " ".join(f'(pin {q(p)} (uuid {U()}))' for p in pins)
        xmin, xmax, ymin, ymax = ext or (-2.54, 2.54, -2.54, 2.54)
        if len(pins) <= 2 and (xmax - xmin) < 3:          # small vertical 2-pin part: text on the right
            rx, ry, vx, vy, rj = round(x + xmax + 1.0, 2), round(y - 0.7, 2), round(x + xmax + 1.0, 2), round(y + 1.5, 2), " (justify left)"
        else:                                              # text above / below the body
            rx, ry, vx, vy, rj = x, round(y - ymax - 1.8, 2), x, round(y - ymin + 1.8, 2), ""
        extra = "".join(f'\n    (property {q(k)} {q(v)} (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))'
                        for k, v in props.items() if v)
        self.items.append(
            f'(symbol (lib_id {q(lib_id)}) (at {x} {y} 0) (unit {unit}) (in_bom yes) (on_board yes) (dnp no) (uuid {U()})\n'
            f'    (property "Reference" {q(ref)} (at {rx} {ry} 0) (effects (font (size 1.27 1.27)){rj}))\n'
            f'    (property "Value" {q(value)} (at {vx} {vy} 0) (effects (font (size 1.27 1.27)){rj}))\n'
            f'    (property "Footprint" {q(fp)} (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))\n'
            f'    (property "Datasheet" "" (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide)){extra}\n'
            f'    {pin_s}\n'
            f'    (instances (project {q(self.c.name)} (path "/{self.root}" (reference {q(ref)}) (unit {unit}))))\n'
            f')')

    # ------------------------------------------------------------------ one placed unit with stubs + labels
    def place_unit(self, part, sym, lib_id, unit, pins_u, x, y, ref=None, value=None, fp=None, props=None):
        self._lib(lib_id, sym)
        self.symbol_inst(lib_id, ref or part.ref, value or part.value, fp if fp is not None else part.fp, x, y, unit,
                         [p[0] for p in pins_u], props or {}, ext=body_extent(sym, unit))
        for num, name, etype, px, py, ang, _u in pins_u:
            sx, sy = round(x + px, 4), round(y - py, 4)
            net = part.pins.get(num) if part else None
            if part is None:
                continue
            if net is None:
                self.noconn(sx, sy)
                continue
            out = (int(ang) + 180) % 360
            dx, dy = {0: (STUB, 0), 180: (-STUB, 0), 90: (0, -STUB), 270: (0, STUB)}[out]
            self.wire(sx, sy, round(sx + dx, 4), round(sy + dy, 4))
            self.label(net, round(sx + dx, 4), round(sy + dy, 4), out)

    # ------------------------------------------------------------------ layout
    def cell_size(self, part, sym, unit):
        xmin, xmax, ymin, ymax = body_extent(sym, unit)
        pins_u = sym_units(sym)[unit]
        lab = lambda side: max([len(part.pins.get(p[0]) or "") for p in pins_u if (int(p[5]) + 180) % 360 == side] or [0])
        wl = lab(180) * 1.3 + (STUB + 2 if lab(180) else 0)
        wr = lab(0) * 1.3 + (STUB + 2 if lab(0) else 0)
        ht = STUB + 2 if any((int(p[5]) + 180) % 360 == 90 for p in pins_u) else 0
        hb = STUB + 2 if any((int(p[5]) + 180) % 360 == 270 for p in pins_u) else 0
        tu = max([len(part.pins.get(p[0]) or "") for p in pins_u if (int(p[5]) + 180) % 360 in (90, 270)] or [0]) * 1.3
        return (xmax - xmin) + wl + wr + 6, (ymax - ymin) + ht + hb + tu + 7, (xmin, xmax, ymin, ymax, wl, ht, tu)

    def build(self):
        page_w = {"A4": 297, "A3": 420, "A2": 594, "A1": 841, "A0": 1189}[self.paper]
        page_h = {"A4": 210, "A3": 297, "A2": 420, "A1": 594, "A0": 841}[self.paper]
        margin = 12.7
        col_x = margin
        col_w_max = 0.0
        y = margin + 10
        blocks = self.c.blocks or [""]
        for blk in blocks:
            parts = [p for p in self.c.parts if p.block == blk]
            cells = []
            for p in parts:
                sym = p.symbol()
                lib_id = f"{p.lib}:{p.sym}"
                for unit, pins_u in sym_units(sym).items():
                    w, h, ext = self.cell_size(p, sym, unit)
                    cells.append((p, sym, lib_id, unit, pins_u, w, h, ext))
            # block width: widest of (sqrt-ish packing) but at least the widest cell
            area = sum(c[5] * c[6] for c in cells)
            bw = max(max(c[5] for c in cells), min(260.0, (area ** 0.5) * 1.5))
            # shelf packing
            rows, row, rw, rh = [], [], 0.0, 0.0
            for cl in sorted(cells, key=lambda c: -c[6]):
                if rw + cl[5] > bw and row:
                    rows.append((row, rh)); row, rw, rh = [], 0.0, 0.0
                row.append(cl); rw += cl[5]; rh = max(rh, cl[6])
            if row:
                rows.append((row, rh))
            bh = sum(r[1] for r in rows) + 12
            if y + bh > page_h - margin - 30 and y > margin + 10:
                col_x += col_w_max + 10
                col_w_max = 0.0
                y = margin + 10
            bx0, by0 = snap(col_x), snap(y)
            self.text(blk, bx0 + 2, by0 + 6, 2.54)
            cy = by0 + 10
            for row, rh in rows:
                cx = bx0 + 2
                for (p, sym, lib_id, unit, pins_u, w, h, ext) in row:
                    xmin, xmax, ymin, ymax, wl, ht, tu = ext
                    x = snap(cx + wl - xmin + 2)
                    yy = snap(cy + ht + tu * 0.5 + ymax + 3)
                    self.place_unit(p, sym, lib_id, unit, pins_u, x, yy,
                                    props={"MPN": p.mpn, "Note": p.note})
                    self.placed_units.append((p.ref, unit, x, yy))
                    cx += w
                cy += rh
            self.rect(bx0, by0, snap(bx0 + bw + 4), snap(by0 + bh))
            col_w_max = max(col_w_max, bw + 4)
            y = by0 + bh + 8
        self.used_width = col_x + col_w_max
        # power flags, in a strip at the bottom
        fx = margin + 5
        fy = snap(page_h - margin - 12)
        flag = kisym.resolve("power", "PWR_FLAG")
        self.text("Power flags (ERC)", margin + 2, fy - 8, 1.8)
        for net in sorted(self.c.power_nets):
            self.flag_n += 1
            ref = f"#FLG{self.flag_n:02d}"
            fake = type("P", (), {})()
            fake.pins = {"1": net}
            fake.ref = ref
            self.place_unit(fake, flag, "power:PWR_FLAG", 1, sym_units(flag)[0], snap(fx), fy,
                            ref=ref, value="PWR_FLAG", fp="")
            fx += max(18, len(net) * 1.5 + 12)
        return self

    def write(self, path):
        today = datetime.date.today().isoformat()
        libs = "\n  ".join(self.lib.values())
        body = "\n  ".join(self.items)
        s = (f'(kicad_sch (version 20230121) (generator eeschema)\n'
             f'  (uuid {self.root})\n'
             f'  (paper {q(self.paper)})\n'
             f'  (title_block (title {q(self.c.title)}) (date {q(today)}) (rev {q(self.rev)}) (company {q(self.company)})\n'
             f'    (comment 1 "Generated by hardware/gen/build.py from hardware/gen/boards.py - edit the Python, not this file")\n'
             f'    (comment 2 "Firmware pin map: firmware/inc/board_pins.h (same source: hardware/gen/pinmap.py)"))\n'
             f'  (lib_symbols\n  {libs}\n  )\n'
             f'  {body}\n'
             f'  (sheet_instances (path "/" (page "1")))\n'
             f')\n')
        with open(path, "w") as fh:
            fh.write(s)
