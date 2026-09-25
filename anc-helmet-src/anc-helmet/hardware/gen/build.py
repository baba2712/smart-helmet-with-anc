#!/usr/bin/env python3
"""Generate every hardware output from boards.py.

    python3 build.py sch      schematics (+ netlist round-trip check, PDF)
    python3 build.py pcb      PCBs (placement, routing via Freerouting, zones, DRC, fab outputs)
    python3 build.py all

Outputs land in hardware/<board>/ (KiCad project) and hardware/<board>/fab/.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from boards import BOARDS          # noqa: E402
from sch_writer import SchWriter   # noqa: E402
import kisym                       # noqa: E402

PAPER = {"main-board": "A1", "satellite-board": "A3", "mic-board": "A4"}


def parse_netlist(path):
    """kicad-cli sexpr netlist -> {net: set((ref, pin))}"""
    root = kisym.parse(open(path).read())
    nets = {}
    for e in root:
        if isinstance(e, list) and e[0] == "nets":
            for n in e[1:]:
                name = [x for x in n if isinstance(x, list) and x[0] == "name"][0][1].strip('"')
                nodes = set()
                for x in n:
                    if isinstance(x, list) and x[0] == "node":
                        ref = [y for y in x if isinstance(y, list) and y[0] == "ref"][0][1].strip('"')
                        pin = [y for y in x if isinstance(y, list) and y[0] == "pin"][0][1].strip('"')
                        nodes.add((ref, pin))
                nets[name.lstrip("/")] = nodes
    return nets


def write_project(d, name):
    pro = os.path.join(d, f"{name}.kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w") as fh:
            fh.write('{\n  "meta": {"filename": "%s.kicad_pro", "version": 1},\n  "board": {},\n  "schematic": {},\n'
                     '  "sheets": [],\n  "text_variables": {}\n}\n' % name)


def build_sch(name):
    circ = BOARDS[name]()
    d = os.path.join(HW, name)
    os.makedirs(os.path.join(d, "fab"), exist_ok=True)
    write_project(d, name)
    sch = os.path.join(d, f"{name}.kicad_sch")
    w = SchWriter(circ, paper=PAPER[name]).build()
    w.write(sch)
    # round trip: KiCad reads our schematic back and exports the netlist it sees
    net = os.path.join(d, "fab", f"{name}.net")
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadsexpr", "-o", net, sch],
                   check=True, capture_output=True)
    got = parse_netlist(net)
    want = {k: set(v) for k, v in circ.nets().items()}
    # KiCad names unlabeled nets itself; compare as sets of connection groups
    got_groups = {frozenset(v) for k, v in got.items() if not any(r.startswith("#") for r, _ in v)}
    got_groups = {frozenset((r, p) for r, p in g if not r.startswith("#")) for g in got_groups}
    want_groups = {frozenset(v) for v in want.values()}
    nc = {(p.ref, pin) for p in circ.parts for pin, n in p.pins.items() if n is None}
    got_groups = {g for g in got_groups if not (len(g) == 1 and next(iter(g)) in nc)}
    missing = want_groups - got_groups
    extra = got_groups - want_groups
    # nets that only exist because of power flags
    flag_only = [k for k, v in got.items() if all(r.startswith("#") for r, _ in v)]
    subprocess.run(["kicad-cli", "sch", "export", "pdf", "-o", os.path.join(d, "fab", f"{name}-schematic.pdf"), sch],
                   check=True, capture_output=True)
    status = "OK" if not missing and not extra and not flag_only else "MISMATCH"
    print(f"[sch] {name}: {len(circ.parts)} parts, {len(want)} nets, {w.label_count} labels -> netlist round-trip {status}")
    for g in list(missing)[:5]:
        print("   missing in schematic:", sorted(g))
    for g in list(extra)[:5]:
        print("   unexpected in schematic:", sorted(g))
    for k in flag_only:
        print("   power flag on a net with no parts:", k)
    return status == "OK"


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    ok = True
    if what in ("sch", "all"):
        for n in BOARDS:
            ok &= build_sch(n)
    if what in ("pcb", "all"):
        import pcb_build
        boards = sys.argv[2:] or list(BOARDS)
        for n in boards:
            pcb_build.stage_place(n)
            pcb_build.stage_route(n, 40)
            b, unr, rpt = pcb_build.stage_finish(n)
            print(f"[pcb] {n}: unrouted connections {unr}, DRC report {os.path.relpath(rpt, HW)}", flush=True)
            pcb_build.stage_fab(n)
            ok &= (unr == 0)
    sys.exit(0 if ok else 1)
