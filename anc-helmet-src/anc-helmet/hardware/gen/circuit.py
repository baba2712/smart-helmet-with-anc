"""Minimal circuit description language.

A Circuit is a list of Parts; each Part maps symbol pin numbers to net names.
From this one description we generate:
    * the KiCad schematic (sch_writer.py)      - label-connected, ERC-able, printable
    * the PCB with footprints + nets (pcb_build.py)
    * the BOM / pick-and-place metadata
and we round-trip check the schematic by exporting its netlist with kicad-cli
and comparing it against this description.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import kisym


@dataclass
class Part:
    ref: str
    lib: str
    sym: str
    value: str
    fp: str
    pins: dict            # pin number (str) -> net name, or None for no-connect
    mpn: str = ""
    block: str = ""
    dnp: bool = False
    note: str = ""
    side: str = "F"       # F/B placement side

    def symbol(self):
        return kisym.resolve(self.lib, self.sym)


@dataclass
class Circuit:
    name: str
    title: str
    parts: list = field(default_factory=list)
    blocks: list = field(default_factory=list)   # ordered block names for sheet layout
    power_nets: set = field(default_factory=set)

    def add(self, p: Part):
        assert all(q.ref != p.ref for q in self.parts), f"duplicate ref {p.ref}"
        sym_pins = {pp[0] for pp in kisym.pins(p.symbol())}
        extra = set(p.pins) - sym_pins
        assert not extra, f"{p.ref}: pins {extra} not in symbol {p.lib}:{p.sym}"
        missing = sym_pins - set(p.pins)
        assert not missing, f"{p.ref} ({p.sym}): unassigned pins {sorted(missing)} (use None for NC)"
        if p.block and p.block not in self.blocks:
            self.blocks.append(p.block)
        self.parts.append(p)
        return p

    def nets(self):
        n = {}
        for p in self.parts:
            for pin, net in p.pins.items():
                if net is not None:
                    n.setdefault(net, []).append((p.ref, pin))
        return n

    def check(self):
        """Design-rule sanity on the description itself."""
        problems = []
        for net, conns in self.nets().items():
            if len(conns) < 2:
                problems.append(f"net {net} has a single connection {conns}")
        return problems


# ---------------------------------------------------------------- part helpers
FP_R0402 = "Resistor_SMD:R_0402_1005Metric"
FP_C0402 = "Capacitor_SMD:C_0402_1005Metric"
FP_C0603 = "Capacitor_SMD:C_0603_1608Metric"
FP_C0805 = "Capacitor_SMD:C_0805_2012Metric"
FP_R0603 = "Resistor_SMD:R_0603_1608Metric"


class Builder:
    """Convenience wrappers that auto-number references."""

    def __init__(self, circ: Circuit):
        self.c = circ
        self.n = {}
        self.block = ""

    def _ref(self, prefix):
        self.n[prefix] = self.n.get(prefix, 0) + 1
        return f"{prefix}{self.n[prefix]}"

    def R(self, a, b, value, fp=FP_R0402, mpn="", note=""):
        return self.c.add(Part(self._ref("R"), "Device", "R", value, fp, {"1": a, "2": b}, mpn or f"{value} 1% {fp[-13:-9]}",
                               self.block, note=note))

    def C(self, a, b, value, fp=None, diel="X7R", note=""):
        if fp is None:
            fp = FP_C0402
        return self.c.add(Part(self._ref("C"), "Device", "C", value, fp, {"1": a, "2": b},
                               f"{value} {diel}", self.block, note=note))

    def FB(self, a, b, value="220R@100MHz", mpn="BLM18PG221SN1D"):
        return self.c.add(Part(self._ref("FB"), "Device", "FerriteBead_Small", value,
                               "Inductor_SMD:L_0603_1608Metric", {"1": a, "2": b}, mpn, self.block))

    def TP(self, net, label=None):
        return self.c.add(Part(self._ref("TP"), "Connector", "TestPoint", label or net,
                               "TestPoint:TestPoint_Pad_D1.0mm", {"1": net}, "", self.block))

    def part(self, prefix, lib, sym, value, fp, pins, mpn="", **kw):
        return self.c.add(Part(self._ref(prefix), lib, sym, value, fp, pins, mpn, self.block, **kw))
