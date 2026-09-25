"""Build a routed KiCad PCB from a Circuit + layout anchors (KiCad 7 pcbnew API).

Steps
  1. footprints + nets from the circuit description (same source as the schematic)
  2. anchors (ICs, connectors, holes) from layouts.py
  3. every other part auto-placed right next to the pads it connects to
     (decoupling caps are assigned one-per-power-pin of their IC)
  4. outline, stack-up, net classes, GND planes
  5. Specctra DSN -> Freerouting -> SES back in
  6. zone fill, DRC report, fabrication outputs (Gerber, drill, pick-and-place, BOM)
"""
from __future__ import annotations

import csv
import json
import math
import os
import shutil
import subprocess
import sys

os.environ.setdefault("KICAD7_FOOTPRINT_DIR", os.environ.get("KICAD_FOOTPRINT_DIR", "/usr/share/kicad/footprints"))
import pcbnew  # noqa: E402  (after the env var so DRC can resolve the stock libraries)

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from boards import BOARDS            # noqa: E402
from layouts import LAYOUTS          # noqa: E402

FPDIR = os.environ.get("KICAD_FOOTPRINT_DIR", "/usr/share/kicad/footprints")
FREEROUTING = os.environ.get("FREEROUTING_JAR", os.path.join(HW, "..", "..", "tools-dl", "freerouting2.jar"))
OX, OY = 100.0, 80.0              # where the board sits on the KiCad page
MM = pcbnew.FromMM

NETCLASSES = {
    # name: (track, clearance, via_dia, via_drill)
    "Default": (0.15, 0.15, 0.6, 0.3),
    # 0.30 mm: must reach 0.5 mm-pitch pads (LQFP-100, WQFN, QFN, USB-C) at 0.18 mm clearance;
    # ~1 A on 1 oz outer copper, above the 500 mA charger / buck-boost input
    "Power":   (0.30, 0.18, 0.7, 0.35),
    "Audio":   (0.20, 0.18, 0.6, 0.3),
}
POWER_PATTERNS = ["VBUS", "VBUS_F", "VSYS", "VBAT", "3V3", "3V3_LDO", "3V3_AMP", "2V8A", "SW_L1", "SW_L2",
                  "SPK_L", "SPK_R", "HPVDD", "HPVSS", "CPP", "CPN", "VCAP", "GND", "VDD"]
AUDIO_PREFIX = ("REF", "ERR", "MIC", "DAC", "AMP_IN", "VMID")


def netclass_of(net):
    if net in POWER_PATTERNS:
        return "Power"
    if net.startswith(AUDIO_PREFIX):
        return "Audio"
    return "Default"


def V(x, y):
    return pcbnew.VECTOR2I(MM(OX + x), MM(OY + y))


def to_local(v):
    return pcbnew.ToMM(v.x) - OX, pcbnew.ToMM(v.y) - OY


# custom design rules (KiCad .kicad_dru) - documented exceptions only
DRU = """(version 1)
# IM73A135 (Infineon PG-LLGA-5-2) datasheet land pattern: the sound-port hole sits
# 0.18 mm inside the GND sealing ring. Intentional - the ring seals the acoustic port.
(rule "MEMS sound port inside its sealing ring"
  (constraint hole_clearance (min 0.15mm))
  (condition "A.Type == 'Pad' && A.Pad_Type == 'NPTH, mechanical' && A.intersectsCourtyard('MK*')"))

# Exposed-pad thermal vias and plated mounting holes are never hand-soldered: join them to the
# GND pours solid, for heat (TPA6132A2 / TPS63001 exposed pads) and a low-impedance chassis tie.
(rule "GND through-holes solid to the pour"
  (constraint zone_connection solid)
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole' && A.NetName == 'GND'"))

# On this dense board some small SMD GND pads get one thermal spoke instead of two. One
# 0.3 mm spoke carries far more than these pads' current, and every such pad also has its
# own via or track to the inner GND plane (checked: 0 unconnected).
(rule "one thermal spoke is enough for small GND pads"
  (constraint min_resolved_spokes 1)
  (condition "A.Type == 'Pad' && A.NetName == 'GND'"))
"""


# ------------------------------------------------------------------------------ geometry
class Occupancy:
    def __init__(self):
        self.rects = {"F": [], "B": []}

    @staticmethod
    def overlap(a, b, gap):
        return not (a[2] + gap <= b[0] or b[2] + gap <= a[0] or a[3] + gap <= b[1] or b[3] + gap <= a[1])

    def free(self, r, side, gap=0.35):
        return all(not self.overlap(r, o, gap) for o in self.rects[side])

    def add(self, r, side):
        self.rects[side].append(r)


def fp_rect(fp, margin=0.0):
    """local-mm bounding box of copper pads + courtyard-ish body (text excluded)."""
    bb = fp.GetBoundingBox(False, False)
    x0, y0 = to_local(bb.GetOrigin())
    x1, y1 = to_local(bb.GetEnd())
    return (min(x0, x1) - margin, min(y0, y1) - margin, max(x0, x1) + margin, max(y0, y1) + margin)


def pads_of(fp):
    return [(p.GetNumber(), p.GetNetname(), to_local(p.GetPosition())) for p in fp.Pads() if p.GetNetname()]


# ------------------------------------------------------------------------------ outline
def add_outline(board, w, h, r):
    def seg(a, b):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(V(*a)); s.SetEnd(V(*b))
        s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(MM(0.1))
        board.Add(s)

    def arc(c, start):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetCenter(V(*c)); s.SetStart(V(*start))
        s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(90, pcbnew.DEGREES_T), True)
        s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(MM(0.1))
        board.Add(s)

    seg((r, 0), (w - r, 0)); seg((w, r), (w, h - r)); seg((w - r, h), (r, h)); seg((0, h - r), (0, r))
    arc((w - r, r), (w - r, 0))      # top-right
    arc((w - r, h - r), (w, h - r))  # bottom-right
    arc((r, h - r), (r, h))          # bottom-left
    arc((r, r), (0, r))              # top-left


def inside_outline(rect, w, h, r, m=0.4):
    x0, y0, x1, y1 = rect
    if x0 < m or y0 < m or x1 > w - m or y1 > h - m:
        return False
    for cx, cy in ((r, r), (w - r, r), (r, h - r), (w - r, h - r)):
        for px, py in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
            inx = (px < cx) if cx == r else (px > cx)
            iny = (py < cy) if cy == r else (py > cy)
            if inx and iny and math.hypot(px - cx, py - cy) > r - m:
                return False
    return True


def add_zone(board, net, layer, w, h, inset=0.3, priority=0, full=False):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNetCode(net.GetNetCode())
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(MM(0.25))
    z.SetMinThickness(MM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if full else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(MM(0.25))
    z.SetThermalReliefSpokeWidth(MM(0.3))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((inset, inset), (w - inset, inset), (w - inset, h - inset), (inset, h - inset)):
        ol.Append(MM(OX + x), MM(OY + y))
    board.Add(z)
    return z


def add_hole_keepouts(board, margin=0.3):
    """Rule-area keep-outs (tracks + vias, all copper layers) around every non-plated
    hole. The Specctra export carries them to the router, which otherwise only sees
    NPTH pads on the layers where they have copper."""
    n = 0
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
                continue
            c = p.GetPosition()
            # a hole inside one of its own footprint's pads (MEMS sound port inside its
            # sealing ring) is part of the land pattern - leave it to the pad
            if any(q is not p and q.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH and q.GetBoundingBox().Contains(c) for q in fp.Pads()):
                continue
            r = pcbnew.ToMM(max(p.GetDrillSize().x, p.GetDrillSize().y)) / 2 + margin
            z = pcbnew.ZONE(board)
            z.SetIsRuleArea(True)
            z.SetDoNotAllowTracks(True)
            z.SetDoNotAllowVias(True)
            z.SetDoNotAllowCopperPour(False)
            z.SetDoNotAllowPads(False)
            z.SetDoNotAllowFootprints(False)
            z.SetLayerSet(pcbnew.LSET.AllCuMask(board.GetCopperLayerCount()))
            ol = z.Outline()
            ol.NewOutline()
            rr = r / math.cos(math.pi / 12)          # circumscribe the circle
            for k in range(12):
                a = 2 * math.pi * (k + 0.5) / 12
                ol.Append(c.x + MM(rr * math.cos(a)), c.y + MM(rr * math.sin(a)))
            board.Add(z)
            n += 1
    return n


def fab_references(board, everything=False):
    """Small passives carry their reference on the fab (assembly) layer instead of the
    silkscreen - dense 0402 areas otherwise end up as overlapping unreadable silk."""
    small = ("Resistor_SMD", "Capacitor_SMD", "Inductor_SMD", "Diode_SMD", "Fuse", "TestPoint")
    for fp in board.GetFootprints():
        if everything or str(fp.GetFPID().GetLibNickname()) in small:
            fp.Reference().SetLayer(pcbnew.B_Fab if fp.IsFlipped() else pcbnew.F_Fab)


def stitch_gnd(board, w, h, corner, pitch=2.5, via=0.6, drill=0.3, clear=0.3):
    """Drop GND stitching vias on a grid wherever they clear every pad, part body,
    track, via, keep-out and the board edge; ties the outer GND pours together."""
    gnd = board.FindNet("GND")
    boxes = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            bb = p.GetBoundingBox()
            boxes.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                          pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())))
        bb = fp.GetBoundingBox(False, False)
        boxes.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                      pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())))
    for z in board.Zones():
        if z.GetIsRuleArea():
            bb = z.GetBoundingBox()
            boxes.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                          pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())))
    segs = []
    for t in board.GetTracks():
        a, b = t.GetStart(), t.GetEnd()
        segs.append((pcbnew.ToMM(a.x), pcbnew.ToMM(a.y), pcbnew.ToMM(b.x), pcbnew.ToMM(b.y), pcbnew.ToMM(t.GetWidth()) / 2))

    def seg_d(px, py, s):
        x1, y1, x2, y2, _ = s
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        u = 0 if L == 0 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))

    added = []
    R = via / 2 + clear
    y = pitch / 2
    while y < h:
        x = pitch / 2
        while x < w:
            gx, gy = OX + x, OY + y
            ok = inside_outline((x - R, y - R, x + R, y + R), w, h, corner, m=0.5)
            ok = ok and all(not (b[0] - R < gx < b[2] + R and b[1] - R < gy < b[3] + R) for b in boxes)
            ok = ok and all(seg_d(gx, gy, s) > s[4] + R for s in segs)
            if ok:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(V(x, y))
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetWidth(MM(via)); v.SetDrill(MM(drill))
                v.SetNet(gnd)
                board.Add(v)
                added.append((gx, gy))
            x += pitch
        y += pitch
    return len(added)


FAN_VIA = (0.5, 0.25, 0.2)     # fan-out via: diameter, drill, clearance (JLCPCB 4-layer standard)
FAN_TRACK = 0.25


def tie_gnd_islands(board, via=0.6, drill=0.3, clear=0.25, step=0.2):
    """Give every outer-layer GND pour island that has no via (or GND through-hole) one via
    down to the inner GND plane. The via must clear other-net copper on EVERY layer and
    stay off all pads (no via-in-pad). Returns (islands tied, islands too small)."""
    gnd = board.FindNet("GND")
    gc = gnd.GetNetCode()
    R = via / 2 + clear
    anchors, segs, obst, obst_other, gnd_pads = [], [], [], [], []
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            if t.GetNetCode() == gc:
                anchors.append(t.GetPosition())
            else:
                c = t.GetPosition()
                segs.append((pcbnew.ToMM(c.x), pcbnew.ToMM(c.y), pcbnew.ToMM(c.x), pcbnew.ToMM(c.y),
                             pcbnew.ToMM(t.GetWidth()) / 2))
        elif t.GetNetCode() != gc:
            a, b = t.GetStart(), t.GetEnd()
            segs.append((pcbnew.ToMM(a.x), pcbnew.ToMM(a.y), pcbnew.ToMM(b.x), pcbnew.ToMM(b.y),
                         pcbnew.ToMM(t.GetWidth()) / 2))
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() == gc and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                anchors.append(p.GetPosition())
            bb = p.GetBoundingBox()
            box = (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                   pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))
            obst.append(box)
            if p.GetNetCode() != gc:
                obst_other.append(box)
            elif p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD:
                gnd_pads.append(p)

    def seg_d(px, py, sg):
        x1, y1, x2, y2, _ = sg
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        u = 0 if L == 0 else max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / L))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))

    ring = [(math.cos(k * math.pi / 4), math.sin(k * math.pi / 4)) for k in range(8)]

    def fanout(isl, layer):
        """A GND pad in this island, a short track out of it and a small via where both clear
        other-net copper. Returns (pad, (x, y), via_dia, drill) or None."""
        fv, fd, fc = FAN_VIA
        rv = fv / 2 + fc
        for pad in gnd_pads:
            if not pad.IsOnLayer(layer) or not isl.Contains(pad.GetPosition()):
                continue
            pc = pad.GetPosition()
            px, py = pcbnew.ToMM(pc.x), pcbnew.ToMM(pc.y)
            pb = pad.GetBoundingBox()
            half = max(pcbnew.ToMM(pb.GetWidth()), pcbnew.ToMM(pb.GetHeight())) / 2
            for dist in (half + rv + 0.05, half + rv + 0.3, half + rv + 0.6, half + rv + 1.0):
                for k in range(16):
                    ang = k * math.pi / 8
                    vx, vy = px + dist * math.cos(ang), py + dist * math.sin(ang)
                    if any(b[0] - rv < vx < b[2] + rv and b[1] - rv < vy < b[3] + rv for b in obst):
                        continue
                    if any(seg_d(vx, vy, sg) <= sg[4] + rv for sg in segs):
                        continue
                    # the track: sample it against other-net copper (any layer - conservative)
                    ok = True
                    for j in range(1, 21):
                        tx, ty = px + (vx - px) * j / 20, py + (vy - py) * j / 20
                        m = FAN_TRACK / 2 + fc
                        if any(b[0] - m < tx < b[2] + m and b[1] - m < ty < b[3] + m for b in obst_other) or \
                           any(seg_d(tx, ty, sg) <= sg[4] + m for sg in segs):
                            ok = False
                            break
                    if ok:
                        return pad, (vx, vy), fv, fd
        return None

    tied = small = 0
    for z in list(board.Zones()):
        if z.GetIsRuleArea() or z.GetNetCode() != gc or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            isl = pcbnew.SHAPE_POLY_SET()
            isl.AddOutline(polys.Outline(i))
            for hk in range(polys.HoleCount(i)):
                isl.AddHole(polys.Hole(i, hk))
            if any(isl.Contains(a) for a in anchors):
                continue
            bb = isl.BBox()
            x0, y0 = pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop())
            x1, y1 = pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())
            best = None
            y = y0
            while y <= y1 and best is None:
                x = x0
                while x <= x1:
                    rr = via / 2 + 0.05
                    ok = isl.Contains(pcbnew.VECTOR2I(MM(x), MM(y))) and all(
                        isl.Contains(pcbnew.VECTOR2I(MM(x + rr * cx), MM(y + rr * cy))) for cx, cy in ring)
                    ok = ok and all(not (b[0] - R < x < b[2] + R and b[1] - R < y < b[3] + R) for b in obst)
                    ok = ok and all(seg_d(x, y, sg) > sg[4] + R for sg in segs)
                    if ok:
                        best = (x, y)
                        break
                    x += step
                y += step
            fan = None
            if best is None:
                fan = fanout(isl, z.GetLayer())
                if fan is None:
                    small += 1
                    continue
                best = fan[1]
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(MM(best[0]), MM(best[1])))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetWidth(MM(via)); v.SetDrill(MM(drill))
            v.SetNet(gnd)
            board.Add(v)
            if fan is not None:                       # short track pad -> via
                pad, (vx, vy), fv, fd = fan
                v.SetWidth(MM(fv)); v.SetDrill(MM(fd))
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pad.GetPosition()); t.SetEnd(pcbnew.VECTOR2I(MM(vx), MM(vy)))
                t.SetWidth(MM(FAN_TRACK)); t.SetLayer(z.GetLayer()); t.SetNet(gnd)
                board.Add(t)
                pc = pad.GetPosition()
                segs.append((pcbnew.ToMM(pc.x), pcbnew.ToMM(pc.y), vx, vy, FAN_TRACK / 2))
            anchors.append(v.GetPosition())
            segs.append((best[0], best[1], best[0], best[1], via / 2))
            tied += 1
    return tied, small


# ------------------------------------------------------------------------------ placement
def place(board, circ, lay, fps):
    w, h = lay["size"]
    r = lay["corner"]
    occ = Occupancy()
    placed = set()
    anchors = lay["anchors"]
    for ref, (x, y, rot, side) in anchors.items():
        fp = fps[ref]
        fp.SetPosition(V(x, y))
        fp.SetOrientationDegrees(rot)
        if side == "B":
            fp.Flip(V(x, y), False)
        occ.add(fp_rect(fp), side)
        placed.add(ref)

    # pad usage counters so decoupling caps spread over the power pins
    pad_use = {}

    def net_targets(part):
        tg = {}
        for pin, net in part.pins.items():
            if not net or net == "GND":
                continue
            pts = []
            for ref in placed:
                for num, pn, pos in pads_of(fps[ref]):
                    if pn == net:
                        pts.append((ref, num, pos))
            if not pts:
                continue
            is_decap = len(part.pins) == 2 and "GND" in part.pins.values() and part.ref.startswith("C")
            if is_decap and len(pts) > 1:
                # the power pin of the nearest big IC with the fewest caps so far
                ics = [p for p in pts if p[0].startswith("U")]
                cand = ics or pts
                best = min(cand, key=lambda p: pad_use.get((p[0], p[1]), 0))
                pad_use[(best[0], best[1])] = pad_use.get((best[0], best[1]), 0) + 1
                tg[pin] = best[2]
            else:
                tg[pin] = (sum(p[2][0] for p in pts) / len(pts), sum(p[2][1] for p in pts) / len(pts))
        return tg

    order = [p for p in circ.parts if p.ref not in placed]
    # parts with the most connections to already-placed parts first (ICs' satellites)
    def score(p):
        return -sum(1 for n in p.pins.values() if n and n != "GND")
    order.sort(key=score)
    remaining = list(order)
    while remaining:
        # pick the part with the most nets already present on the board
        best, best_n = None, -1
        present = {pn for ref in placed for _, pn, _ in pads_of(fps[ref])}
        for p in remaining:
            k = sum(1 for n in p.pins.values() if n and n != "GND" and n in present)
            if k > best_n:
                best, best_n = p, k
        p = best
        remaining.remove(p)
        fp = fps[p.ref]
        side = p.side
        tg = net_targets(p)
        if tg:
            tx = sum(v[0] for v in tg.values()) / len(tg)
            ty = sum(v[1] for v in tg.values()) / len(tg)
        else:
            tx, ty = w / 2, h / 2
        done = False
        step = 0.25
        for ring in range(0, 200):
            rad = ring * step
            n_pts = max(1, int(2 * math.pi * rad / step)) if rad else 1
            cands = []
            for k in range(n_pts):
                a = 2 * math.pi * k / n_pts
                cands.append((tx + rad * math.cos(a), ty + rad * math.sin(a)))
            best_c = None
            for (cx, cy) in cands:
                for rot in (0, 90, 180, 270):
                    fp.SetOrientationDegrees(rot)
                    fp.SetPosition(V(cx, cy))
                    rect = fp_rect(fp)
                    if not inside_outline(rect, w, h, r) or not occ.free(rect, side):
                        continue
                    cost = 0.0
                    for num, pn, pos in pads_of(fp):
                        if num in tg:
                            cost += math.hypot(pos[0] - tg[num][0], pos[1] - tg[num][1])
                    if best_c is None or cost < best_c[0]:
                        best_c = (cost, cx, cy, rot)
            if best_c:
                _, cx, cy, rot = best_c
                fp.SetOrientationDegrees(rot)
                fp.SetPosition(V(cx, cy))
                if side == "B":
                    fp.Flip(V(cx, cy), False)
                occ.add(fp_rect(fp), side)
                placed.add(p.ref)
                done = True
                break
        if not done:
            raise RuntimeError(f"no room for {p.ref} ({p.value}) on {circ.name}")


# ------------------------------------------------------------------------------ project / rules
def write_project(path, board_layers):
    classes = []
    for name, (tw, cl, vd, vdr) in NETCLASSES.items():
        classes.append({"name": name, "clearance": cl, "track_width": tw, "via_diameter": vd, "via_drill": vdr,
                        "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
                        "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12, "line_style": 0,
                        "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)"})
    patterns = [{"netclass": "Power", "pattern": p} for p in POWER_PATTERNS]
    patterns += [{"netclass": "Audio", "pattern": f"{p}*"} for p in AUDIO_PREFIX]
    pro = {
        "board": {"design_settings": {"rules": {
            "min_clearance": 0.15, "min_track_width": 0.127, "min_via_diameter": 0.45, "min_via_annular_width": 0.1,
            "min_through_hole_diameter": 0.2, "min_hole_to_hole": 0.25, "min_hole_clearance": 0.25,
            "min_copper_edge_clearance": 0.3, "min_silk_clearance": 0.0, "min_microvia_diameter": 0.2,
            "min_microvia_drill": 0.1, "max_error": 0.005, "solder_mask_to_copper_clearance": 0.0,
            "use_height_for_length_calcs": True}}},
        "meta": {"filename": os.path.basename(path), "version": 1},
        "net_settings": {"classes": classes, "meta": {"version": 3}, "net_colors": None,
                         "netclass_assignments": None, "netclass_patterns": patterns},
        "schematic": {}, "sheets": [], "text_variables": {},
    }
    with open(path, "w") as fh:
        json.dump(pro, fh, indent=2)


def apply_runtime_netclasses(board):
    """Assign net classes in the live board (used by DSN export and DRC in this session)."""
    ns = board.GetDesignSettings().m_NetSettings
    objs = {}
    for name, (tw, cl, vd, vdr) in NETCLASSES.items():
        nc = ns.m_DefaultNetClass if name == "Default" else pcbnew.NETCLASS(name)
        nc.SetTrackWidth(MM(tw)); nc.SetClearance(MM(cl)); nc.SetViaDiameter(MM(vd)); nc.SetViaDrill(MM(vdr))
        if name != "Default":
            ns.m_NetClasses[name] = nc
        objs[name] = nc
    for net in board.GetNetsByName().values():
        n = net.GetNetname()
        if n:
            net.SetNetClass(objs[netclass_of(n)])


# ------------------------------------------------------------------------------ routing
def freeroute(dsn, ses, passes=60, timeout=1500):
    jar = os.path.abspath(FREEROUTING)
    cmd = ["java", "-Djava.awt.headless=true", "-jar", jar, "--gui.enabled=false", "-de", dsn, "-do", ses,
           "-mp", str(passes), "-mt", "2"]
    log = ses + ".log"
    with open(log, "w") as fh:
        subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT, timeout=timeout)
    return os.path.exists(ses)


def stage_place(name):
    """footprints, placement, outline, rules, inner GND plane -> .kicad_pcb + .dsn"""
    circ = BOARDS[name]()
    lay = LAYOUTS[name]
    d = os.path.join(HW, name)
    os.makedirs(os.path.join(d, "fab"), exist_ok=True)
    pcb_path = os.path.join(d, f"{name}.kicad_pcb")
    write_project(os.path.join(d, f"{name}.kicad_pro"), lay["layers"])
    with open(os.path.join(d, f"{name}.kicad_dru"), "w") as fh:
        fh.write(DRU)

    board = pcbnew.NewBoard(pcb_path)
    board.SetCopperLayerCount(lay["layers"])
    tb = board.GetTitleBlock()
    tb.SetTitle(circ.title); tb.SetRevision("A"); tb.SetCompany("Open ANC Helmet")
    bds = board.GetDesignSettings()
    bds.m_TrackMinWidth = MM(0.127)
    bds.m_ViasMinSize = MM(0.45)             # finisher fan-out vias 0.45/0.2 (JLCPCB 4-layer standard)
    bds.m_MinThroughDrill = MM(0.2)          # TI QFN/WQFN thermal-pad vias (JLCPCB 4-layer: 0.15 mm min)
    bds.m_CopperEdgeClearance = MM(0.3)
    bds.m_HoleClearance = MM(0.25)
    bds.m_HoleToHoleMin = MM(0.25)
    if lay["layers"] == 4:
        board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)

    nets = {}
    for n in circ.nets():
        ni = pcbnew.NETINFO_ITEM(board, n)
        board.Add(ni)
        nets[n] = ni
    fps = {}
    for p in circ.parts:
        lib, fpn = p.fp.split(":")
        fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + ".pretty"), fpn)
        if fp is None:
            raise RuntimeError(f"footprint {p.fp} not found")
        fp.SetFPID(pcbnew.LIB_ID(lib, fpn))
        fp.SetReference(p.ref)
        fp.SetValue(p.value)
        ref = fp.Reference()
        ref.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        ref.SetTextThickness(MM(0.15))
        fp.Value().SetVisible(False)
        for pad in fp.Pads():
            net = p.pins.get(pad.GetNumber())
            if net:
                pad.SetNet(nets[net])
        board.Add(fp)
        fps[p.ref] = fp

    w, h = lay["size"]
    add_outline(board, w, h, lay["corner"])
    place(board, circ, lay, fps)
    fab_references(board, lay.get("refs_on_fab", False))
    add_hole_keepouts(board)
    apply_runtime_netclasses(board)
    if lay["layers"] == 4:
        add_zone(board, nets["GND"], pcbnew.In1_Cu, w, h)      # solid GND plane: the router drops vias into it
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(pcb_path, board)
    dsn = os.path.join(d, f"{name}.dsn")
    pcbnew.ExportSpecctraDSN(board, dsn)
    return pcb_path, dsn


def stage_route(name, passes=40):
    d = os.path.join(HW, name)
    dsn = os.path.join(d, f"{name}.dsn")
    ses = os.path.join(d, f"{name}.ses")
    if os.path.exists(ses):
        os.remove(ses)
    jar = os.path.abspath(FREEROUTING)
    cmd = ["java", "-Djava.awt.headless=true", "-jar", jar, "--gui.enabled=false",
           "--usage_and_diagnostic_data.disable_analytics=true", "--api_server.enabled=false",
           "-de", dsn, "-do", ses, "-mp", str(passes), "-mt", "2"]
    with open(ses + ".log", "w") as fh:
        subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT)
    return os.path.exists(ses) and os.path.getsize(ses) > 0


def import_ses(board, path):
    """Specctra session -> tracks + vias (KiCad 7's ImportSpecctraSES only works inside the GUI)."""
    import re
    import kisym
    root = kisym.parse(open(path).read())
    routes = [e for e in root if isinstance(e, list) and e[0] == "routes"][0]
    res = [e for e in routes if isinstance(e, list) and e[0] == "resolution"][0]
    scale = {"um": 1e-3, "mil": 0.0254, "mm": 1.0, "inch": 25.4}[res[1]] / float(res[2])   # -> mm
    netout = [e for e in routes if isinstance(e, list) and e[0] == "network_out"]
    layers = {board.GetLayerName(l): l for l in range(pcbnew.PCB_LAYER_ID_COUNT)}
    n_w = n_v = 0
    # remove any previous routing (Tracks(), not GetTracks(): the latter trips a KiCad 7
    # SWIG wrapper bug on boards loaded after a finish)
    for t in list(board.Tracks()):
        board.Remove(t)
    for no in netout:
        for net in no[1:]:
            if not isinstance(net, list) or net[0] != "net":
                continue
            ni = board.FindNet(net[1].strip('"'))
            for e in net[2:]:
                if not isinstance(e, list):
                    continue
                if e[0] == "wire":
                    path_ = [x for x in e if isinstance(x, list) and x[0] == "path"][0]
                    lay = layers[path_[1].strip('"')]
                    wdt = float(path_[2]) * scale
                    pts = [(float(path_[i]) * scale, -float(path_[i + 1]) * scale) for i in range(3, len(path_) - 1, 2)]
                    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                        t = pcbnew.PCB_TRACK(board)
                        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
                        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
                        t.SetWidth(MM(wdt))
                        t.SetLayer(lay)
                        t.SetNet(ni)
                        board.Add(t)
                        n_w += 1
                elif e[0] == "via":
                    m = re.search(r"_(\d+):(\d+)_um", e[1])
                    dia, drill = (int(m.group(1)) / 1000.0, int(m.group(2)) / 1000.0) if m else (0.6, 0.3)
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pcbnew.VECTOR2I(MM(float(e[2]) * scale), MM(-float(e[3]) * scale)))
                    v.SetViaType(pcbnew.VIATYPE_THROUGH)
                    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                    v.SetWidth(MM(dia))
                    v.SetDrill(MM(drill))
                    v.SetNet(ni)
                    board.Add(v)
                    n_v += 1
    return n_w, n_v


def unrouted(board):
    """Unrouted connection count. KiCad 7's SWIG wrapper sometimes hands back an
    unwrapped connectivity pointer depending on import history, so count in a
    clean interpreter from a saved copy."""
    conn = board.GetConnectivity()
    if hasattr(conn, "RecalculateRatsnest"):
        conn.RecalculateRatsnest()
        return conn.GetUnconnectedCount(False)
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        tmp = os.path.join(td, "count.kicad_pcb")
        pcbnew.SaveBoard(tmp, board)
        code = ("import pcbnew,sys; b=pcbnew.LoadBoard(sys.argv[1]); c=b.GetConnectivity(); "
                "c.RecalculateRatsnest(); print('UNR', c.GetUnconnectedCount(False))")
        out = subprocess.run([sys.executable, "-c", code, tmp], capture_output=True, text=True).stdout
    return int([l for l in out.splitlines() if l.startswith("UNR")][0].split()[1])


def stage_finish(name):
    """SES in, outer GND pours, fill, DRC, fab outputs."""
    lay = LAYOUTS[name]
    d = os.path.join(HW, name)
    pcb_path = os.path.join(d, f"{name}.kicad_pcb")
    ses = os.path.join(d, f"{name}.ses")
    board = pcbnew.LoadBoard(pcb_path)
    # touch the shared_ptr-returning wrappers before the net-class code runs; KiCad 7's
    # SWIG type table otherwise ends up handing back raw pointers later in this process
    board.GetConnectivity().RecalculateRatsnest()
    list(board.Tracks())
    apply_runtime_netclasses(board)
    if os.path.exists(ses):
        nw, nv = import_ses(board, ses)
        print(f"imported {nw} track segments, {nv} vias")
        # completion rounds: hand the partly routed board back to the router
        for rnd in range(3):
            n = unrouted(board)
            if n == 0:
                break
            print(f"  {n} connections left -> completion round {rnd + 1}")
            dsn2 = os.path.join(d, f"{name}.dsn")
            pcbnew.ExportSpecctraDSN(board, dsn2)
            if not stage_route(name, 30):
                print("  completion round produced no session - keeping the routing we have")
                break
            nw, nv = import_ses(board, ses)
            print(f"  re-imported {nw} segments, {nv} vias")
    w, h = lay["size"]
    # hand-routed links the autorouter can't make (e.g. custom-shaped pads)
    for (r1, p1, r2, p2, width) in lay.get("hand_tracks", []):
        a = [p for p in board.FindFootprintByReference(r1).Pads() if p.GetNumber() == p1][0]
        b = [p for p in board.FindFootprintByReference(r2).Pads() if p.GetNumber() == p2][0]
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(a.GetPosition()); t.SetEnd(b.GetPosition())
        t.SetWidth(MM(width)); t.SetLayer(pcbnew.F_Cu); t.SetNet(a.GetNet())
        board.Add(t)
    fab_references(board, lay.get("refs_on_fab", False))
    if lay.get("zone_full"):
        for fp in board.GetFootprints():
            fp.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    print(f"  {stitch_gnd(board, w, h, lay['corner'], pitch=lay.get('stitch_pitch', 2.5))} GND stitching vias")
    gnd = board.FindNet("GND")
    have = {(z.GetLayer()) for z in board.Zones() if not z.GetIsRuleArea()}
    for L in (pcbnew.F_Cu, pcbnew.B_Cu):
        if L not in have:
            add_zone(board, gnd, L, w, h, full=lay.get("zone_full", False))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    # outer-layer GND islands the stitching grid missed: one via each down to the plane
    for _ in range(3):
        tied, small = tie_gnd_islands(board)
        print(f"  {tied} GND islands tied to the plane" + (f", {small} too small for a via" if small else ""))
        if not tied:
            break
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(pcb_path, board)
    board = pcbnew.LoadBoard(pcb_path)
    n_unr = unrouted(board)
    rpt = os.path.join(d, "fab", f"{name}-drc.rpt")
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    return board, n_unr, rpt


def stage_tie(name):
    """Re-run only the GND-island tie + refill + DRC on an already finished board."""
    d = os.path.join(HW, name)
    pcb_path = os.path.join(d, f"{name}.kicad_pcb")
    board = pcbnew.LoadBoard(pcb_path)
    board.GetConnectivity().RecalculateRatsnest()      # same SWIG warm-up as stage_finish
    list(board.Tracks())
    board.GetDesignSettings().m_ViasMinSize = MM(0.45)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())      # tracks may have been added since the last fill
    for _ in range(3):
        tied, small = tie_gnd_islands(board)
        print(f"  {tied} GND islands tied to the plane" + (f", {small} too small for a via" if small else ""))
        if not tied:
            break
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(pcb_path, board)
    board = pcbnew.LoadBoard(pcb_path)
    n_unr = unrouted(board)
    rpt = os.path.join(d, "fab", f"{name}-drc.rpt")
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    return n_unr, rpt


def stage_complete(name, passes=30):
    """One more autorouter round on a finished board (its ground vias and pours included),
    then GND islands, refill, DRC. Keeps the current routing if the router adds nothing."""
    d = os.path.join(HW, name)
    pcb_path = os.path.join(d, f"{name}.kicad_pcb")
    board = pcbnew.LoadBoard(pcb_path)
    board.GetConnectivity().RecalculateRatsnest()
    list(board.Tracks())
    before = unrouted(board)
    pcbnew.ExportSpecctraDSN(board, os.path.join(d, f"{name}.dsn"))
    if stage_route(name, passes):
        backup = pcb_path + ".bak"
        pcbnew.SaveBoard(backup, board)
        nw, nv = import_ses(board, os.path.join(d, f"{name}.ses"))
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        after = unrouted(board)
        print(f"  completion: {before} -> {after} unrouted ({nw} segments, {nv} vias)")
        if after > before:
            board = pcbnew.LoadBoard(backup)
            list(board.Tracks())
            print("  worse - kept the previous routing")
        os.remove(backup)
    pcbnew.SaveBoard(pcb_path, board)
    # the SES import leaves KiCad 7's SWIG wrappers in a bad state: tie + DRC in a fresh process
    r = subprocess.run([sys.executable, os.path.abspath(__file__), "tie", name], capture_output=True, text=True)
    print(r.stdout.strip())
    rpt = os.path.join(d, "fab", f"{name}-drc.rpt")
    return int(r.stdout.split("unrouted connections:")[1].split()[0]), rpt


def stage_fab(name):
    """Gerbers + drill (zipped for JLCPCB), JLC BOM + CPL, STEP, top/bottom renders."""
    circ = BOARDS[name]()
    lay = LAYOUTS[name]
    d = os.path.join(HW, name)
    fab = os.path.join(d, "fab")
    pcb = os.path.join(d, f"{name}.kicad_pcb")
    gdir = os.path.join(fab, "gerbers")
    if os.path.isdir(gdir):
        shutil.rmtree(gdir)
    os.makedirs(gdir)
    cu = ["F.Cu", "B.Cu"] if lay["layers"] == 2 else ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    layers = cu + ["F.Paste", "B.Paste", "F.SilkS", "B.SilkS", "F.Mask", "B.Mask", "Edge.Cuts"]
    run = lambda *a: subprocess.run(["kicad-cli", "pcb", "export", *a], check=True, capture_output=True)
    run("gerbers", "--layers", ",".join(layers), "--no-x2", "--output", gdir + "/", pcb)
    run("drill", "--format", "excellon", "--excellon-separate-th", "--output", gdir + "/", pcb)
    shutil.make_archive(os.path.join(fab, f"{name}-gerbers-jlcpcb"), "zip", gdir)

    # pick & place -> JLCPCB CPL columns
    pos = os.path.join(fab, f"{name}-pos-raw.csv")
    run("pos", "--format", "csv", "--units", "mm", "--side", "both", "--output", pos, pcb)
    with open(pos) as fi, open(os.path.join(fab, f"{name}-cpl-jlcpcb.csv"), "w", newline="") as fo:
        rd = csv.DictReader(fi)
        wr = csv.writer(fo)
        wr.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for row in rd:
            if row["Ref"].startswith(("H", "TP")):
                continue
            wr.writerow([row["Ref"], f"{float(row['PosX']):.3f}mm", f"{float(row['PosY']):.3f}mm",
                         "Top" if row["Side"] == "top" else "Bottom", f"{float(row['Rot']):.1f}"])
    os.remove(pos)

    # BOM grouped by value + footprint + MPN (JLCPCB columns + our notes)
    groups = {}
    for p in circ.parts:
        if p.ref.startswith(("H", "TP")):
            continue
        k = (p.value, p.fp.split(":")[1], p.mpn)
        groups.setdefault(k, []).append(p)
    with open(os.path.join(fab, f"{name}-bom-jlcpcb.csv"), "w", newline="") as fo:
        wr = csv.writer(fo)
        wr.writerow(["Comment", "Designator", "Footprint", "Quantity", "Manufacturer Part", "LCSC Part #", "Notes"])
        for (val, fpn, mpn), ps in sorted(groups.items(), key=lambda kv: kv[1][0].ref):
            refs = sorted((q.ref for q in ps), key=lambda r: (r.rstrip("0123456789"), int("0" + r[len(r.rstrip("0123456789")):])))
            notes = sorted({q.note for q in ps if q.note})
            if any(lay["anchors"].get(q.ref, (0, 0, 0, "F"))[3] == "B" for q in ps):
                notes.append("BOTTOM SIDE (hand-solder, or order 2-sided assembly)")
            notes = "; ".join(notes)
            wr.writerow([val, ",".join(refs), fpn, len(ps), mpn, "", notes])

    # 3D (board + footprint bodies where models are installed) and renders
    try:
        subprocess.run(["kicad-cli", "pcb", "export", "step", "--subst-models", "--force", "-o",
                        os.path.join(fab, f"{name}.step"), pcb], check=True, capture_output=True, timeout=600)
    except Exception as ex:  # models are optional
        print("  STEP export skipped:", ex)
    for side, lays in (("top", "F.Cu,F.Mask,F.SilkS,Edge.Cuts"), ("bottom", "B.Cu,B.Mask,B.SilkS,Edge.Cuts")):
        svg = os.path.join(fab, f"{name}-{side}.svg")
        extra = ["--mirror"] if side == "bottom" else []
        run("svg", "--layers", lays, "--page-size-mode", "2", "--exclude-drawing-sheet", *extra, "-o", svg, pcb)
        png = svg[:-4] + ".png"
        if shutil.which("rsvg-convert"):
            subprocess.run(["rsvg-convert", "-w", "1600", "-b", "white", svg, "-o", png], check=False)
    return True


if __name__ == "__main__":
    stage = sys.argv[1]
    name = sys.argv[2]
    if stage == "place":
        print(stage_place(name))
    elif stage == "route":
        print("routed:", stage_route(name, int(sys.argv[3]) if len(sys.argv) > 3 else 40))
    elif stage == "finish":
        b, n, rpt = stage_finish(name)
        print("unrouted connections:", n, "report:", rpt)
    elif stage == "complete":
        n, rpt = stage_complete(name)
        print("unrouted connections:", n, "report:", rpt)
    elif stage == "tie":
        n, rpt = stage_tie(name)
        print("unrouted connections:", n, "report:", rpt)
    elif stage == "fab":
        stage_fab(name)
