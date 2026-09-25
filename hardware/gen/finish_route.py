"""Finish the connections the autorouter leaves: a small grid maze router (A*) on a
finished board, working directly in the .kicad_pcb (KiCad 7 pcbnew API).

Two jobs:
  * a two-pad signal connection (e.g. AMP_INLN U10.1 -> C51.1): A* between the pads over
    F.Cu / In2.Cu / B.Cu with vias, avoiding other-net copper with clearance;
  * a GND pad whose pour island has no path to the inner GND plane: A* from the pad to the
    nearest spot where a via fits, then a via down to the plane.

    python3 finish_route.py main-board [REF.PAD ...] [--islands]   # SIGNALS/RIPUP + GND fan-outs
    python3 finish_route.py main-board --gnd-only REF.PAD ... [--islands]
    python3 finish_route.py main-board --rip NET --fan REF.PAD         # whole-net rip-up + re-route
    python3 finish_route.py main-board --rip-local NET --fan REF.PAD   # rip only segments near the pad
The --rip modes save only when everything re-connects. Afterwards run `pcb_build.py tie <board>`
in a fresh process (refill + DRC).

How the main board was finished (from 11 unrouted after autorouting):
    finish_route.py main-board                          AMP_INLN (+ SPK_L rip-up / re-route)
    finish_route.py main-board --gnd-only C44.2 R1.2 U2.2 R15.2 --islands
    finish_route.py main-board --rip DACL_OUT --fan R15.2
    finish_route.py main-board --rip DACL_B --fan U9.4
    finish_route.py main-board --rip ERRL_SKA --fan R22.2
    finish_route.py main-board --rip-local 2V8A --fan U5.19
"""
from __future__ import annotations

import heapq
import math
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
MM = pcbnew.FromMM
TO = pcbnew.ToMM

GRID = 0.05                 # mm
WINDOW = 8.0                # search window margin around the endpoints (mm)
VIA = (0.45, 0.2)           # diameter, drill (board min drill 0.2 mm; JLCPCB 4-layer ok)
MASK_WEB = 0.1              # via edge to a same-net pad: solder-mask web, not electrical clearance
VIA_COST = 30               # in grid steps
LAYERS = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]   # In1 is the GND plane: never routed on

# per-board signal connections to finish: (net, (ref, pad), (ref, pad), track width, clearance)
SIGNALS = {
    "main-board": [("AMP_INLN", ("U10", "1"), ("C51", "1"), 0.15, 0.18)],   # 0.15: board min 0.127
}
# two-pad nets to rip up first and re-route after the signals above (they wall a pad in):
# (net, (ref, pad), (ref, pad), width, clearance)
RIPUP = {
    # SPK_L leaves U10.16 and curls round U10.1, sealing AMP_INLN in on F.Cu (EP copper below)
    "main-board": [("SPK_L", ("U10", "16"), ("J8", "1"), 0.20, 0.18)],   # <= 56 mA peak: 0.2 mm is ample
}


class Obstacles:
    """Other-net copper as inflatable primitives, per layer (vias/PTH on all)."""

    def __init__(self, board, net_code, box):
        x0, y0, x1, y1 = box
        self.segs = {L: [] for L in LAYERS + [pcbnew.In1_Cu]}
        self.rects = {L: [] for L in LAYERS + [pcbnew.In1_Cu]}
        self.circles = []                                        # through all layers
        near = lambda ax, ay, bx, by, r: not (max(ax, bx) + r < x0 or min(ax, bx) - r > x1 or
                                              max(ay, by) + r < y0 or min(ay, by) - r > y1)
        for t in board.Tracks():
            if t.GetNetCode() == net_code:
                continue
            if t.Type() == pcbnew.PCB_VIA_T:
                c = t.GetPosition()
                r = TO(t.GetWidth()) / 2
                if near(TO(c.x), TO(c.y), TO(c.x), TO(c.y), r):
                    self.circles.append((TO(c.x), TO(c.y), r))
            else:
                a, b = t.GetStart(), t.GetEnd()
                hw = TO(t.GetWidth()) / 2
                if t.GetLayer() in self.segs and near(TO(a.x), TO(a.y), TO(b.x), TO(b.y), hw):
                    self.segs[t.GetLayer()].append((TO(a.x), TO(a.y), TO(b.x), TO(b.y), hw))
        for fp in board.GetFootprints():
            for p in fp.Pads():
                bb = p.GetBoundingBox()
                r = (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))
                if not near(r[0], r[1], r[2], r[3], 0):
                    continue
                if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                    if p.GetNetCode() != net_code or p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                        for L in self.rects:
                            self.rects[L].append(r)
                    continue
                if p.GetNetCode() == net_code:
                    continue
                for L in self.rects:
                    if p.IsOnLayer(L):
                        self.rects[L].append(r)
        # rule areas (keep-outs) block tracks and vias on every layer
        for z in board.Zones():
            if z.GetIsRuleArea():
                bb = z.GetBoundingBox()
                r = (TO(bb.GetLeft()), TO(bb.GetTop()), TO(bb.GetRight()), TO(bb.GetBottom()))
                if near(r[0], r[1], r[2], r[3], 0):
                    for L in self.rects:
                        self.rects[L].append(r)

    @staticmethod
    def _seg_d(px, py, s):
        x1, y1, x2, y2, _ = s
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        u = 0 if L == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
        return math.hypot(px - (x1 + u * dx), py - (y1 + u * dy))

    def free(self, x, y, layer, r):
        """A disc of radius r at (x, y) on this layer clears all other-net copper."""
        for s in self.segs[layer]:
            if self._seg_d(x, y, s) < s[4] + r:
                return False
        for (a, b, c, d) in self.rects[layer]:
            if a - r < x < c + r and b - r < y < d + r:
                return False
        for (cx, cy, cr) in self.circles:
            if math.hypot(x - cx, y - cy) < cr + r:
                return False
        return True

    def via_free(self, x, y, clear, plane_ok):
        r = VIA[0] / 2 + clear
        layers = LAYERS + ([] if plane_ok else [pcbnew.In1_Cu])
        return all(self.free(x, y, L, r) for L in layers)


def astar(obs, start, goal_fn, box, width, clear, start_layer, allow_vias=True):
    """Grid A* from start (x, y) on start_layer. goal_fn(x, y, layer) -> bool.
    Returns [(x, y, layer), ...] or None."""
    x0, y0, x1, y1 = box
    r = width / 2 + clear
    sx, sy = start
    nx, ny = int((x1 - x0) / GRID) + 1, int((y1 - y0) / GRID) + 1
    li = {L: i for i, L in enumerate(LAYERS)}
    to_xy = lambda i, j: (x0 + i * GRID, y0 + j * GRID)
    si, sj = round((sx - x0) / GRID), round((sy - y0) / GRID)
    start_n = (si, sj, li[start_layer])
    free_cache = {}

    def ok(i, j, k):
        key = (i, j, k)
        if key not in free_cache:
            x, y = to_xy(i, j)
            free_cache[key] = obs.free(x, y, LAYERS[k], r)
        return free_cache[key]

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    openq = [(0.0, 0.0, start_n)]
    came = {start_n: None}
    cost = {start_n: 0.0}
    # the first few steps may be inside the (same-net) start pad's clearance of nothing: allow them
    grace = int(0.35 / GRID)
    while openq:
        _, g, n = heapq.heappop(openq)
        if g > cost.get(n, 1e18):
            continue
        i, j, k = n
        x, y = to_xy(i, j)
        if n != start_n and goal_fn(x, y, LAYERS[k]):
            path = []
            while n is not None:
                path.append((*to_xy(n[0], n[1]), LAYERS[n[2]]))
                n = came[n]
            return path[::-1]
        steps = [(i + di, j + dj, k, c) for di, dj, c in moves]
        if allow_vias:
            steps += [(i, j, kk, VIA_COST) for kk in range(len(LAYERS)) if kk != k]
        for (a, b, kk, c) in steps:
            if not (0 <= a < nx and 0 <= b < ny):
                continue
            near_start = abs(a - si) <= grace and abs(b - sj) <= grace and kk == start_n[2]
            if kk != k:
                vx, vy = to_xy(a, b)
                if not obs.via_free(vx, vy, clear, plane_ok=False):
                    continue
            elif not near_start and not ok(a, b, kk):
                continue
            m = (a, b, kk)
            ng = g + c
            if ng < cost.get(m, 1e18):
                cost[m] = ng
                came[m] = n
                heapq.heappush(openq, (ng, ng, m))
    return None


def simplify(path):
    """Merge collinear grid steps into segments; keep layer changes as via points."""
    out = [path[0]]
    for p in path[1:]:
        out.append(p)
        if len(out) >= 3 and out[-1][2] == out[-2][2] == out[-3][2]:
            (ax, ay, _), (bx, by, _), (cx, cy, _) = out[-3], out[-2], out[-1]
            if abs((bx - ax) * (cy - by) - (by - ay) * (cx - bx)) < 1e-9:
                out.pop(-2)
    return out


def commit(board, net, path, width, via_at_end=False):
    for (ax, ay, la), (bx, by, lb) in zip(path, path[1:]):
        if la != lb:
            add_via(board, net, ax, ay)
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(ax), MM(ay)))
        t.SetEnd(pcbnew.VECTOR2I(MM(bx), MM(by)))
        t.SetWidth(MM(width)); t.SetLayer(la); t.SetNet(net)
        board.Add(t)
    if via_at_end:
        add_via(board, net, path[-1][0], path[-1][1])


def add_via(board, net, x, y):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetWidth(MM(VIA[0])); v.SetDrill(MM(VIA[1]))
    v.SetNet(net)
    board.Add(v)


def pad_of(board, ref, num):
    return [p for p in board.FindFootprintByReference(ref).Pads() if p.GetNumber() == num][0]


def route_signal(board, netname, a, b, width, clear):
    pa, pb = pad_of(board, *a), pad_of(board, *b)
    net = pa.GetNet()
    ca, cb = pa.GetPosition(), pb.GetPosition()
    ax, ay, bx, by = TO(ca.x), TO(ca.y), TO(cb.x), TO(cb.y)
    box = (min(ax, bx) - WINDOW, min(ay, by) - WINDOW, max(ax, bx) + WINDOW, max(ay, by) + WINDOW)
    obs = Obstacles(board, net.GetNetCode(), box)
    bbb = pb.GetBoundingBox()
    goal = lambda x, y, L: (L == pcbnew.F_Cu and TO(bbb.GetLeft()) <= x <= TO(bbb.GetRight())
                            and TO(bbb.GetTop()) <= y <= TO(bbb.GetBottom()))
    path = astar(obs, (ax, ay), goal, box, width, clear, pcbnew.F_Cu)
    if path is None:
        return False
    path = simplify(path)
    path[-1] = (bx, by, path[-1][2])
    commit(board, net, path, width)
    return True


def fanout_gnd(board, pad, width=0.15, clear=0.18):
    """A* from a GND pad to the nearest spot a via fits (clear on every routing layer)."""
    net = pad.GetNet()
    c = pad.GetPosition()
    sx, sy = TO(c.x), TO(c.y)
    box = (sx - WINDOW, sy - WINDOW, sx + WINDOW, sy + WINDOW)
    obs = Obstacles(board, net.GetNetCode(), box)
    pbb = pad.GetBoundingBox()
    hx, hy = TO(pbb.GetWidth()) / 2, TO(pbb.GetHeight()) / 2
    # no via in any pad: same-net pads only need a solder-mask web (other nets are obstacles)
    same = [(TO(q.GetBoundingBox().GetLeft()), TO(q.GetBoundingBox().GetTop()),
             TO(q.GetBoundingBox().GetRight()), TO(q.GetBoundingBox().GetBottom()))
            for fp in board.GetFootprints() for q in fp.Pads() if q.GetNetCode() == net.GetNetCode()]
    rw = VIA[0] / 2 + MASK_WEB
    # existing GND vias / through-holes already reach the plane: ending on one needs no new via
    anchors = [(TO(t.GetPosition().x), TO(t.GetPosition().y)) for t in board.Tracks()
               if t.Type() == pcbnew.PCB_VIA_T and t.GetNetCode() == net.GetNetCode()]
    anchors += [(TO(q.GetPosition().x), TO(q.GetPosition().y)) for fp in board.GetFootprints() for q in fp.Pads()
                if q.GetNetCode() == net.GetNetCode() and q.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
    anchors = [a for a in anchors if math.hypot(a[0] - sx, a[1] - sy) > 0.05]

    # pour islands on this layer already tied to the plane: running a track into one connects
    layer0 = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    tied = []
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetCode() != net.GetNetCode() or z.GetLayer() != layer0:
            continue
        polys = z.GetFilledPolysList(layer0)
        for i in range(polys.OutlineCount()):
            isl = pcbnew.SHAPE_POLY_SET()
            isl.AddOutline(polys.Outline(i))
            for h in range(polys.HoleCount(i)):
                isl.AddHole(polys.Hole(i, h))
            if isl.Contains(c):
                continue                                   # the pad's own (untied) island
            if any(isl.Contains(pcbnew.VECTOR2I(MM(ax), MM(ay))) for ax, ay in anchors):
                tied.append(isl)
    ring = [(math.cos(k * math.pi / 4), math.sin(k * math.pi / 4)) for k in range(8)]

    def in_tied_pour(x, y):
        # the whole track end (half width + a little) inside a tied pour island
        rr = width / 2 + 0.05
        return any(isl.Contains(pcbnew.VECTOR2I(MM(x), MM(y))) and
                   all(isl.Contains(pcbnew.VECTOR2I(MM(x + rr * a), MM(y + rr * b))) for a, b in ring)
                   for isl in tied)

    def at_anchor(x, y):
        return any(math.hypot(x - ax, y - ay) <= GRID * 1.01 for ax, ay in anchors) or in_tied_pour(x, y)

    def goal(x, y, L):
        if L != pcbnew.F_Cu:
            return False
        if at_anchor(x, y):
            return True
        if any(a - rw < x < cc + rw and b - rw < y < d + rw for (a, b, cc, d) in same):
            return False
        return obs.via_free(x, y, clear, plane_ok=True)

    layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    # the start pad's own island: an existing via's disc sits in a same-net obstacle-free spot,
    # so obstacles never block reaching it
    path = astar(obs, (sx, sy), goal, box, width, clear, layer, allow_vias=False)
    if path is None:
        return False
    path = simplify(path)
    commit(board, net, path, width, via_at_end=not at_anchor(path[-1][0], path[-1][1]))
    return True


def route_net_tree(board, netname, width, clear):
    """Route every pad of a net: grow a tree from its first pad, each step an A* from the
    nearest unconnected pad to any copper already in the tree (tracks, vias or pads)."""
    pads = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetname() == netname]
    if len(pads) < 2:
        return True
    net = pads[0].GetNet()
    xy = lambda p: (TO(p.GetPosition().x), TO(p.GetPosition().y))
    tree_pads = [pads[0]]
    tree_segs = []                                  # (x1, y1, x2, y2, layer)
    left = pads[1:]
    while left:
        left.sort(key=lambda p: min(math.hypot(xy(p)[0] - xy(q)[0], xy(p)[1] - xy(q)[1]) for q in tree_pads))
        p = left.pop(0)
        px, py = xy(p)
        pts = [xy(q) for q in tree_pads] + [(a, b) for a, b, *_ in tree_segs] + [(c, d) for _, _, c, d, _ in tree_segs]
        box = (min(px, *[q[0] for q in pts]) - WINDOW, min(py, *[q[1] for q in pts]) - WINDOW,
               max(px, *[q[0] for q in pts]) + WINDOW, max(py, *[q[1] for q in pts]) + WINDOW)
        obs = Obstacles(board, net.GetNetCode(), box)
        boxes = [(TO(q.GetBoundingBox().GetLeft()), TO(q.GetBoundingBox().GetTop()),
                  TO(q.GetBoundingBox().GetRight()), TO(q.GetBoundingBox().GetBottom()), q) for q in tree_pads]

        def goal(x, y, L):
            for (a, b, c, d, q) in boxes:
                if q.IsOnLayer(L) and a <= x <= c and b <= y <= d:
                    return True
            return any(L == sl and Obstacles._seg_d(x, y, (x1, y1, x2, y2, 0)) <= GRID * 0.75
                       for (x1, y1, x2, y2, sl) in tree_segs)

        layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        path = astar(obs, (px, py), goal, box, width, clear, layer)
        if path is None:
            return False
        path = simplify(path)
        commit(board, net, path, width)
        for (ax, ay, la), (bx, by, lb) in zip(path, path[1:]):
            if la == lb:
                tree_segs.append((ax, ay, bx, by, la))
            else:
                for L in LAYERS:
                    tree_segs.append((ax, ay, ax, ay, L))
        tree_pads.append(p)
    return True


def island_pads(board):
    """GND SMD pads on outer layers inside pour islands that have no via / through-hole."""
    gnd = board.FindNet("GND")
    gc = gnd.GetNetCode()
    anchors = [t.GetPosition() for t in board.Tracks() if t.Type() == pcbnew.PCB_VIA_T and t.GetNetCode() == gc]
    anchors += [p.GetPosition() for fp in board.GetFootprints() for p in fp.Pads()
                if p.GetNetCode() == gc and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
    pads = [p for fp in board.GetFootprints() for p in fp.Pads()
            if p.GetNetCode() == gc and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
    out = []
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetCode() != gc or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            isl = pcbnew.SHAPE_POLY_SET()
            isl.AddOutline(polys.Outline(i))
            for h in range(polys.HoleCount(i)):
                isl.AddHole(polys.Hole(i, h))
            if any(isl.Contains(a) for a in anchors):
                continue
            inside = [p for p in pads if p.IsOnLayer(z.GetLayer()) and isl.Contains(p.GetPosition())]
            if inside:
                out.append(inside)
    return out


def rip_and_fan(name, net_to_rip, fan):
    """Rip up one net, fan a GND pad out through the gap, re-route the net. Saves only if all
    of it worked (exit 0), else leaves the file untouched (exit 1)."""
    pcb = os.path.join(HW, name, f"{name}.kicad_pcb")
    board = pcbnew.LoadBoard(pcb)
    board.GetConnectivity().RecalculateRatsnest()
    list(board.Tracks())
    board.GetDesignSettings().m_ViasMinSize = MM(0.45)
    old = [t for t in board.Tracks() if t.GetNetname() == net_to_rip]
    w = max([TO(t.GetWidth()) for t in old if t.Type() != pcbnew.PCB_VIA_T] or [0.15])
    w = min(w, 0.3)
    for t in old:
        board.Remove(t)
    ref, num = fan.split(".")
    if not fanout_gnd(board, pad_of(board, ref, num)):
        print(f"  rip {net_to_rip}: {fan} still has no path")
        return 1
    for width in (w, 0.15):
        if route_net_tree(board, net_to_rip, width, 0.18):
            pcbnew.SaveBoard(pcb, board)
            print(f"  rip {net_to_rip}: {fan} connected, {net_to_rip} re-routed at {width} mm")
            return 0
    print(f"  rip {net_to_rip}: {fan} connected but {net_to_rip} could not be re-routed")
    return 1


def rip_local_and_fan(name, net_to_rip, fan, radius=1.5):
    """Rip up only the segments of one net near a GND pad, fan the pad out, then re-join the
    net's cut ends (endpoints of the removed chain that were not shared inside it)."""
    pcb = os.path.join(HW, name, f"{name}.kicad_pcb")
    board = pcbnew.LoadBoard(pcb)
    board.GetConnectivity().RecalculateRatsnest()
    list(board.Tracks())
    board.GetDesignSettings().m_ViasMinSize = MM(0.45)
    ref, num = fan.split(".")
    pad = pad_of(board, ref, num)
    cx, cy = TO(pad.GetPosition().x), TO(pad.GetPosition().y)
    near = []
    for t in board.Tracks():
        if t.GetNetname() != net_to_rip or t.Type() == pcbnew.PCB_VIA_T:
            continue
        a, b = t.GetStart(), t.GetEnd()
        sg = (TO(a.x), TO(a.y), TO(b.x), TO(b.y), 0)
        if Obstacles._seg_d(cx, cy, sg) <= radius:
            near.append((t, sg[:4], t.GetLayer(), TO(t.GetWidth())))
    if not near:
        print(f"  local rip {net_to_rip}: nothing near {fan}")
        return 1
    key = lambda x, y, L: (round(x, 3), round(y, 3), L)
    count = {}
    for _, (x1, y1, x2, y2), L, _ in near:
        for k in (key(x1, y1, L), key(x2, y2, L)):
            count[k] = count.get(k, 0) + 1
    ends = [k for k, n in count.items() if n == 1]
    width = max(w for *_, w in near)
    for t, *_ in near:
        board.Remove(t)
    if not fanout_gnd(board, pad):
        print(f"  local rip {net_to_rip}: {fan} still has no path")
        return 1
    net = board.FindNet(net_to_rip)
    tree = [ends[0]]
    segs = []
    for (ex, ey, eL) in ends[1:]:
        box = (min(ex, *[p[0] for p in tree]) - WINDOW, min(ey, *[p[1] for p in tree]) - WINDOW,
               max(ex, *[p[0] for p in tree]) + WINDOW, max(ey, *[p[1] for p in tree]) + WINDOW)
        obs = Obstacles(board, net.GetNetCode(), box)

        def goal(x, y, L):
            return any(L == tL and math.hypot(x - tx, y - ty) <= GRID * 0.75 for tx, ty, tL in tree) or \
                   any(L == sL and Obstacles._seg_d(x, y, (x1, y1, x2, y2, 0)) <= GRID * 0.75
                       for x1, y1, x2, y2, sL in segs)
        path = None
        for w in (width, 0.15):
            path = astar(obs, (ex, ey), goal, box, w, 0.18, eL)
            if path:
                break
        if path is None:
            print(f"  local rip {net_to_rip}: cut end ({ex:.2f}, {ey:.2f}) could not be re-joined")
            return 1
        path = simplify(path)
        path[0] = (ex, ey, path[0][2])
        commit(board, net, path, w)
        for (ax, ay, la), (bx, by, lb) in zip(path, path[1:]):
            if la == lb:
                segs.append((ax, ay, bx, by, la))
        tree.append((ex, ey, eL))
    pcbnew.SaveBoard(pcb, board)
    print(f"  local rip {net_to_rip}: {len(near)} segments near {fan} replaced, {len(ends)} cut ends re-joined, {fan} connected")
    return 0


def main():
    if "--rip-local" in sys.argv:
        a = sys.argv
        sys.exit(rip_local_and_fan(a[1], a[a.index("--rip-local") + 1], a[a.index("--fan") + 1]))
    if "--rip" in sys.argv:
        a = sys.argv
        sys.exit(rip_and_fan(a[1], a[a.index("--rip") + 1], a[a.index("--fan") + 1]))
    name = sys.argv[1]
    pcb = os.path.join(HW, name, f"{name}.kicad_pcb")
    board = pcbnew.LoadBoard(pcb)
    board.GetConnectivity().RecalculateRatsnest()
    list(board.Tracks())
    only_gnd = "--gnd-only" in sys.argv
    if only_gnd:
        sys.argv.remove("--gnd-only")
    ripped = [] if only_gnd else RIPUP.get(name, [])
    for netname, *_ in ripped:
        gone = [t for t in board.Tracks() if t.GetNetname() == netname]
        for t in gone:
            board.Remove(t)
        print(f"  ripped up {netname} ({len(gone)} items)")
    for netname, a, b, w, c in ([] if only_gnd else SIGNALS.get(name, [])) + ripped:
        ok = route_signal(board, netname, a, b, w, c)
        print(f"  {netname} {a[0]}.{a[1]} -> {b[0]}.{b[1]}: {'routed' if ok else 'NO PATH'}")
    for ref, num in [tuple(s.split(".")) for s in sys.argv[2:] if s != "--islands"]:
        ok = fanout_gnd(board, pad_of(board, ref, num))
        print(f"  GND fan-out {ref}.{num}: {'connected' if ok else 'NO PATH'}")
    if "--islands" in sys.argv:
        for group in island_pads(board):
            names = ", ".join(f"{p.GetParent().GetReference()}.{p.GetNumber()}" for p in group)
            ok = any(fanout_gnd(board, p) for p in group)
            print(f"  GND island [{names}]: {'connected' if ok else 'NO PATH'}")
    pcbnew.SaveBoard(pcb, board)


if __name__ == "__main__":
    main()
