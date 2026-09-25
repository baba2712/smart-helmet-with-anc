#!/usr/bin/env python3
"""Host tool for the ANC helmet (USB CDC or a BLE-UART bridge).

    pip install pyserial matplotlib
    python3 anc_tool.py status                 one reading
    python3 anc_tool.py watch                  live table, 1 Hz
    python3 anc_tool.py plot                   live plot: outside vs at-ear dB(A)
    python3 anc_tool.py cal-paths [--seconds 3]    factory speaker-path calibration (quiet room!)
    python3 anc_tool.py cal-mic refl [--db 94]     mic trim with a 1 kHz calibrator
    python3 anc_tool.py log out.csv            download the minute-by-minute exposure log
    python3 anc_tool.py set mu_ff 0.0015       change a setting (then: anc_tool.py raw save)
    python3 anc_tool.py raw "<command>"        send any CLI command, print the reply
    python3 anc_tool.py shell                  interactive terminal

The port is auto-detected by USB VID:PID 1209:0001; override with --port.
"""
import argparse
import json
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

VID, PID = 0x1209, 0x0001


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


class Helmet:
    def __init__(self, port=None, baud=115200, timeout=1.0):
        port = port or find_port()
        if not port:
            sys.exit("helmet not found - plug in USB-C (and power it on), or pass --port")
        self.s = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.1)
        self.s.reset_input_buffer()

    def cmd(self, line, wait_end=None, timeout=2.0):
        """send a line, return the reply lines (until quiet, or until a line starting with wait_end)"""
        self.s.write((line + "\r\n").encode())
        out, t0 = [], time.time()
        while time.time() - t0 < timeout:
            ln = self.s.readline().decode(errors="replace").strip()
            if ln:
                out.append(ln)
                t0 = time.time()
                if wait_end and ln.startswith(wait_end):
                    break
            elif out and not wait_end:
                break
        return out

    def status(self):
        for ln in self.cmd("status"):
            if ln.startswith("{"):
                return json.loads(ln)
        return None


def fmt(st):
    return (f"{st['mode']:8s} ear {st['ear_dba']:5.1f} dB(A)  outside {st['amb_dba']:5.1f}  "
            f"reduction {st['atten_db']:5.1f} dB  dose {st['dose_pct']:6.2f}% (unprotected {st['dose_unprot_pct']:.0f}%)  "
            f"batt {st['vbat']:.2f} V{' chg' if st['chg'] else ''}  cpu {st['cpu_pct']:.0f}%  trips {st['trips']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("what")
    ap.add_argument("args", nargs="*")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--db", type=float, default=94.0)
    a = ap.parse_args()
    h = Helmet(a.port)

    if a.what == "status":
        st = h.status()
        print(json.dumps(st, indent=2) if st else "no reply")
    elif a.what == "watch":
        while True:
            st = h.status()
            if st:
                print(fmt(st), flush=True)
            time.sleep(1)
    elif a.what == "plot":
        import collections
        import matplotlib.pyplot as plt
        n = 300
        amb, ear = collections.deque(maxlen=n), collections.deque(maxlen=n)
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 4))
        la, = ax.plot([], [], label="outside dB(A)")
        le, = ax.plot([], [], label="at the ear dB(A)")
        ax.axhline(85, color="k", ls=":", lw=0.8, label="85 dB(A) 8 h limit")
        ax.set_ylim(40, 110); ax.set_xlabel("seconds"); ax.legend(loc="upper left"); ax.grid(alpha=0.3)
        while plt.fignum_exists(fig.number):
            st = h.status()
            if st:
                amb.append(st["amb_dba"]); ear.append(st["ear_dba"])
                x = list(range(len(amb)))
                la.set_data(x, list(amb)); le.set_data(x, list(ear))
                ax.set_xlim(0, max(n, len(x)))
                ax.set_title(f"{st['mode']}  reduction {st['atten_db']:.1f} dB  dose {st['dose_pct']:.1f} %")
            plt.pause(1.0)
    elif a.what == "cal-paths":
        print("\n".join(h.cmd(f"cal paths {a.seconds:.0f}")))
        time.sleep(a.seconds + 0.5)
        print("\n".join(h.cmd("status", timeout=3)))
    elif a.what == "cal-mic":
        if not a.args:
            sys.exit("which mic: refl errl refr errr")
        print("\n".join(h.cmd(f"cal mic {a.args[0]} {a.db}", timeout=3)))
        time.sleep(1.5)
        print("\n".join(h.cmd("get")))
    elif a.what == "log":
        lines = h.cmd("log", wait_end="END", timeout=5)
        rows = [ln for ln in lines if not ln.startswith("END")]
        if a.args:
            with open(a.args[0], "w") as fh:
                fh.write("\n".join(rows) + "\n")
            print(f"wrote {len(rows) - 1} records to {a.args[0]}")
        else:
            print("\n".join(rows))
    elif a.what == "set":
        print("\n".join(h.cmd("set " + " ".join(a.args))))
    elif a.what == "raw":
        print("\n".join(h.cmd(" ".join(a.args), timeout=3)))
    elif a.what == "shell":
        print("type commands ('help'), ctrl-c to quit")
        try:
            while True:
                line = input("helmet> ")
                print("\n".join(h.cmd(line, timeout=2)))
        except (KeyboardInterrupt, EOFError):
            print()
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
