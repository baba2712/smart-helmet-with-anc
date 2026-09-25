"""Board-vs-cup fit check.

Cross-checks the three sources of truth and the geometry between them:
  - params.scad            (what the printed parts are built around)
  - hardware/gen/layouts.py (what the PCB generator uses)
  - hardware/*/*.kicad_pcb  (what actually gets fabricated)

and then checks that the boards, battery and driver fit the elliptical cup and
that the left-cup stack fits its depth. No KiCad install needed.

    python3 mechanical/check_fit.py        # exit code 1 on any failure
"""
from __future__ import annotations

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "hardware", "gen"))
from layouts import LAYOUTS  # noqa: E402

OX, OY = 100.0, 80.0         # board origin on the KiCad page (pcb_build.py)
TOL = 0.05                   # mm, for "these numbers must match"

# values baked into the .scad files (not in params.scad)
GAP = 0.6                    # insert-to-cup clearance, both inserts
LEFT_RIM = 1.6               # rear-tray rim wall thickness (cup_insert_left.scad)
LEFT_TRAY_T = 1.6
BAFFLE_T = 2.0
FENCE_WALL = 1.2             # battery fence wall
# tallest part on the main board's front face (JST PH side-entry ~4.8 mm, USB-C ~3.3 mm)
MAX_PART_H = 5.0

failures = []
warnings = []


def check(ok, msg, warn=False):
    """warn=True: a shortfall is reported but doesn't fail the run."""
    print(("  ok    " if ok else "  WARN  " if warn else "  FAIL  ") + msg)
    if not ok:
        (warnings if warn else failures).append(msg)


# ------------------------------------------------------------------ parsing
def read_params(path):
    out = {}
    src = re.sub(r"//[^\n]*", "", open(path).read())
    src = src.split("module ")[0]                 # assignments only, not module bodies
    for m in re.finditer(r"(?:^|;)\s*(\$?\w+)\s*=\s*([^;]+)(?=;)", src):
        txt = m.group(2).strip()
        try:
            out[m.group(1)] = eval(txt, {"__builtins__": {}}, {})
        except Exception:
            pass                                  # expressions like $fn etc.
    return out


def read_scad_numbers(path):
    """Top-level 'name = number;' lines of a .scad file (module bodies included - names are unique)."""
    return {m.group(1): float(m.group(2))
            for m in re.finditer(r"^\s*(\w+)\s*=\s*(-?[\d.]+)\s*;", open(path).read(), re.M)}


def tokenize(s):
    return re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+', s)


def sexpr(path):
    toks = tokenize(open(path).read())
    pos = 0

    def rd():
        nonlocal pos
        t = toks[pos]
        pos += 1
        if t == "(":
            lst = []
            while toks[pos] != ")":
                lst.append(rd())
            pos += 1
            return lst
        return t.strip('"')
    return rd()


def kids(node, name):
    return [e for e in node if isinstance(e, list) and e and e[0] == name]


def footprints(pcb):
    """ref -> (x, y, rot, [(pad_type, local_x, local_y)]) in board coordinates."""
    res = {}
    for fp in kids(pcb, "footprint"):
        at = kids(fp, "at")[0]
        x, y = float(at[1]) - OX, float(at[2]) - OY
        rot = float(at[3]) if len(at) > 3 else 0.0
        ref = None
        for p in kids(fp, "property") + kids(fp, "fp_text"):
            if p[1] == "Reference":
                ref = p[2]
            elif p[1] == "reference":
                ref = p[2]
        pads = []
        for pad in kids(fp, "pad"):
            pat = kids(pad, "at")[0]
            pads.append((pad[2], float(pat[1]), float(pat[2])))
        res[ref] = (x, y, rot, pads)
    return res


def placements(pcb):
    """ref -> (x, y, rot, side) as placed in the .kicad_pcb."""
    out = {}
    for fp in kids(pcb, "footprint"):
        at = kids(fp, "at")[0]
        ref = next((p[2] for p in kids(fp, "property") + kids(fp, "fp_text") if p[1] in ("Reference", "reference")), None)
        side = "B" if kids(fp, "layer")[0][1] == "B.Cu" else "F"
        out[ref] = (float(at[1]) - OX, float(at[2]) - OY, float(at[3]) if len(at) > 3 else 0.0, side)
    return out


def rot_matches(want, side, got):
    """layouts.py gives the rotation as seen from the part's own side; KiCad stores a
    flipped (B.Cu) footprint's orientation mirrored, so 90 on B reads -90."""
    w = -want if side == "B" else want
    return abs(((got - w) + 180.0) % 360.0 - 180.0) < 0.1


def pad_abs(fp, px, py):
    x, y, rot, _ = fp
    a = math.radians(rot)                        # KiCad: CCW rotation, y down
    return x + px * math.cos(a) + py * math.sin(a), y - px * math.sin(a) + py * math.cos(a)


def outline_bbox(pcb):
    xs, ys = [], []
    for kind in ("gr_line", "gr_arc"):
        for g in kids(pcb, kind):
            if kids(g, "layer")[0][1] != "Edge.Cuts":
                continue
            for k in ("start", "end", "mid"):
                for p in kids(g, k):
                    xs.append(float(p[1]) - OX)
                    ys.append(float(p[2]) - OY)
    return min(xs), min(ys), max(xs), max(ys)


def outline_arc_radii(pcb):
    """Radius of every Edge.Cuts arc (circle through its start, mid and end points)."""
    radii = []
    for g in kids(pcb, "gr_arc"):
        if kids(g, "layer")[0][1] != "Edge.Cuts":
            continue
        (ax, ay), (bx, by), (cx, cy) = [(float(kids(g, k)[0][1]), float(kids(g, k)[0][2])) for k in ("start", "mid", "end")]
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
        radii.append(math.hypot(ax - ux, ay - uy))
    return radii


# ------------------------------------------------------------------ geometry
def rounded_rect_points(w, h, r, n=90):
    """Perimeter of a w x h rounded rectangle centred on the origin."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90),
                       (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def ellipse_clearance(pts, a, b, n=4000):
    """Smallest distance from any point to an a x b (full axes) ellipse boundary,
    negative if a point lies outside."""
    ring = [(a / 2 * math.cos(2 * math.pi * i / n), b / 2 * math.sin(2 * math.pi * i / n)) for i in range(n)]
    worst = math.inf
    for x, y in pts:
        d = min(math.hypot(x - ex, y - ey) for ex, ey in ring)
        inside = (x / (a / 2)) ** 2 + (y / (b / 2)) ** 2 <= 1.0
        worst = min(worst, d if inside else -d)
    return worst


# ------------------------------------------------------------------ checks
def main():
    P = read_params(os.path.join(HERE, "params.scad"))
    hw = os.path.join(ROOT, "hardware")
    pcbs = {n: sexpr(os.path.join(hw, n, f"{n}.kicad_pcb")) for n in ("main-board", "satellite-board", "mic-board")}

    print("params.scad vs layouts.py vs .kicad_pcb")
    for name, key in (("main-board", "main"), ("satellite-board", "sat")):
        lay = LAYOUTS[name]
        w, h, r = P[f"{key}_w"], P[f"{key}_h"], P[f"{key}_r"]
        check(abs(w - lay["size"][0]) < TOL and abs(h - lay["size"][1]) < TOL and abs(r - lay["corner"]) < TOL,
              f"{name}: size {w} x {h} r{r} matches layouts.py {lay['size'][0]} x {lay['size'][1]} r{lay['corner']}")
        x0, y0, x1, y1 = outline_bbox(pcbs[name])
        check(abs(x0) < TOL and abs(y0) < TOL and abs(x1 - w) < TOL and abs(y1 - h) < TOL,
              f"{name}: Edge.Cuts {x1 - x0:.2f} x {y1 - y0:.2f} mm matches")
        fps = footprints(pcbs[name])
        holes = sorted((k for k in fps if re.fullmatch(r"H\d+", k)), key=lambda k: int(k[1:]))
        want = P[f"{key}_holes"]
        check(len(holes) == len(want), f"{name}: {len(holes)} mounting holes on the PCB, {len(want)} in params.scad")
        for ref, (hx, hy) in zip(holes, want):
            px, py = fps[ref][0], fps[ref][1]
            lx, ly = lay["anchors"][ref][:2]
            check(math.hypot(px - hx, py - hy) < TOL and math.hypot(lx - hx, ly - hy) < TOL,
                  f"{name}: {ref} at ({px:.2f}, {py:.2f}) matches params ({hx}, {hy})")

    # Edge.Cuts corners: four arcs of the layout's corner radius (a bounding box alone can't see it)
    for name in ("main-board", "satellite-board", "mic-board"):
        want = LAYOUTS[name]["corner"]
        radii = outline_arc_radii(pcbs[name])
        check(len(radii) == 4 and all(abs(r - want) < TOL for r in radii),
              f"{name}: Edge.Cuts corners r {', '.join(f'{r:.2f}' for r in radii)} = layouts.py r{want}")

    # every hand-placed anchor (ICs, connectors, UI parts, holes): position, rotation, side
    for name in ("main-board", "satellite-board", "mic-board"):
        placed = placements(pcbs[name])
        bad = []
        for ref, (ax, ay, arot, aside) in LAYOUTS[name]["anchors"].items():
            if ref not in placed:
                bad.append(f"{ref} missing")
                continue
            px, py, prot, pside = placed[ref]
            if math.hypot(px - ax, py - ay) > TOL or pside != aside or not rot_matches(arot, aside, prot):
                bad.append(f"{ref}: layout ({ax}, {ay}) {arot} deg {aside}, board ({px:.2f}, {py:.2f}) {prot:g} deg {pside}")
        check(not bad, f"{name}: all {len(LAYOUTS[name]['anchors'])} anchors match layouts.py "
              "(position, rotation, side)" + ("" if not bad else " - " + "; ".join(bad)))

    # mic board: size + sound port (the NPTH pad of MK1) against the pod's hole
    mic = pcbs["mic-board"]
    x0, y0, x1, y1 = outline_bbox(mic)
    check(abs(x1 - x0 - P["mic_w"]) < TOL and abs(y1 - y0 - P["mic_h"]) < TOL,
          f"mic-board: Edge.Cuts {x1 - x0:.2f} x {y1 - y0:.2f} mm matches params {P['mic_w']} x {P['mic_h']}")
    mk = footprints(mic)["MK1"]
    port = [pad_abs(mk, px, py) for t, px, py in mk[3] if t == "np_thru_hole"]
    check(len(port) == 1, "mic-board: MK1 has one sound-port hole")
    if port:
        check(math.hypot(port[0][0] - P["mic_port"][0], port[0][1] - P["mic_port"][1]) < TOL,
              f"mic-board: sound port at ({port[0][0]:.2f}, {port[0][1]:.2f}) matches mic_port {P['mic_port']}")

    print("\nfit in the cup")
    # the boards sit above the inserts' rim walls, so the cup wall itself is the limit;
    # less than the inserts' own clearance (GAP) is flagged as too tight to rely on
    for name, key in (("main-board", "main"), ("satellite-board", "sat")):
        c = ellipse_clearance(rounded_rect_points(P[f"{key}_w"], P[f"{key}_h"], P[f"{key}_r"]),
                              P["cup_a"], P["cup_b"])
        check(c > 0, f"{name}: inside the {P['cup_a']} x {P['cup_b']} cup")
        if c > 0:
            check(c >= GAP, f"{name}: {c:.2f} mm to the cup wall (want >= {GAP}, the inserts' clearance)", warn=True)
    A, B = P["cup_a"] - 2 * GAP, P["cup_b"] - 2 * GAP          # insert outline
    rim_a, rim_b = A - 2 * LEFT_RIM, B - 2 * LEFT_RIM          # inside the rear tray's rim wall
    bw, bh = P["batt"][0], P["batt"][1]
    fw, fh = bw + 2 * FENCE_WALL, bh + 2 * FENCE_WALL
    c = ellipse_clearance(rounded_rect_points(fw, fh, 0.01), rim_a, rim_b)
    check(c >= 0, f"battery fence {fw:.1f} x {fh:.1f}: {c:.2f} mm inside the tray rim")
    c = min(A, B) / 2 - (P["driver_d"] + 4) / 2
    check(c >= 2.0, f"driver boss d{P['driver_d'] + 4:.1f}: {c:.2f} mm of baffle around it")
    # main-board standoffs (cup_insert_left.scad): the board holes sit over the battery's
    # corners, so each is a pillar outside the battery fence (floor to board) plus a lug
    # bridging over the battery to a boss under the hole
    L_ = read_scad_numbers(os.path.join(HERE, "cup_insert_left.scad"))
    sd, gap_z = L_["standoff_d"], L_["boss_gap"]
    sr = sd / 2
    for (hx, hy) in P["main_holes"]:
        x, y = hx - P["main_w"] / 2, P["main_h"] / 2 - hy
        px = math.copysign(min(abs(x), L_["pillar_x_max"]), x)           # pillar_xy() in the .scad
        py = math.copysign(bh / 2 + FENCE_WALL + sr, y)
        reach = math.hypot(px - x, py - y)
        ring = [(px + sr * math.cos(t / 16 * math.pi), py + sr * math.sin(t / 16 * math.pi)) for t in range(32)]
        check(ellipse_clearance(ring, A, B) >= 0, f"standoff pillar under ({hx}, {hy}) on the tray, inside the cup")
        # pillar edge to the battery pocket (negative = overlap)
        dx, dy = abs(px) - bw / 2, abs(py) - bh / 2
        gap = (math.hypot(max(dx, 0), max(dy, 0)) if max(dx, dy) > 0 else max(dx, dy)) - sr
        check(gap >= 0, f"standoff pillar under ({hx}, {hy}): {gap:.2f} mm clear of the {bw} x {bh} battery pocket")
        # the boss over the battery starts above it; the insert needs its depth
        boss_h = P["standoff_h"] - gap_z
        check(gap_z > 0 and boss_h >= 3.0, f"boss under ({hx}, {hy}): {gap_z} mm above the battery, "
              f"{boss_h:.1f} mm tall for the M2 heat-set insert")
        # 45-degree lug underside: horizontal reach = drop, so it prints without supports
        z_boss = LEFT_TRAY_T + P["batt"][2] + gap_z
        check(z_boss - reach >= LEFT_TRAY_T, f"lug reach {reach:.2f} mm: 45-degree underside starts "
              f"{z_boss - reach:.1f} mm up the pillar (printable without supports)")

    print("\nleft-cup depth")
    stack = [("tray", LEFT_TRAY_T), ("battery", P["batt"][2]), ("standoff", P["standoff_h"]),
             ("main PCB", P["pcb_t"]), ("tallest part", MAX_PART_H), ("baffle", BAFFLE_T),
             ("driver", P["driver_depth"])]
    total = sum(v for _, v in stack)
    print("        " + " + ".join(f"{n} {v:g}" for n, v in stack) + f" = {total:.1f} mm")
    check(total <= P["cup_depth"], f"stack {total:.1f} mm fits cup depth {P['cup_depth']} mm "
          f"({P['cup_depth'] - total:.1f} mm left for ear foam)")

    print()
    if warnings:
        print(f"{len(warnings)} warning(s)")
    if failures:
        print(f"{len(failures)} check(s) failed")
        return 1
    print("all fit checks pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
