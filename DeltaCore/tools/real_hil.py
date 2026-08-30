#!/usr/bin/env python3
"""DeltaCore real hardware-in-the-loop test runner.

Target rig:
  Mega2560 + RAMPS 1.4/MKS MINI compatible pinout
  3 stepper drivers + 3 motors
  3 MAX endstops

The runner deliberately uses the real USB serial transport and the actual
firmware parser/scheduler. It supports a setup diagnostic mode and a repeatable
regression suite with strict PASS/FAIL checks against DeltaCore golden results.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import queue
import re
import sys
import threading
import time
from pathlib import Path
from typing import Iterable, Optional

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except ImportError:
    print("ERROR: pyserial is required. Install with: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)

BAUD = 250000

SHORT_SEGMENT = [
    "M972", "G1 X0 Y0 Z120 F7200",
    "G1 X2 Y0", "G1 X4 Y1", "G1 X6 Y2", "G1 X8 Y4", "G1 X9 Y6", "G1 X10 Y8",
    "G1 X10 Y10", "G1 X9 Y12", "G1 X8 Y14", "G1 X6 Y16", "G1 X4 Y17", "G1 X2 Y18",
    "G1 X0 Y18", "G1 X-2 Y18", "G1 X-4 Y17", "G1 X-6 Y16", "G1 X-8 Y14", "G1 X-9 Y12",
    "G1 X-10 Y10", "G1 X-10 Y8", "G1 X-9 Y6", "G1 X-8 Y4", "G1 X-6 Y2", "G1 X-4 Y1",
    "G1 X-2 Y0", "G1 X0 Y0",
    "G1 X2 Y0", "G1 X4 Y-1", "G1 X6 Y-2", "G1 X8 Y-4", "G1 X9 Y-6", "G1 X10 Y-8",
    "G1 X10 Y-10", "G1 X9 Y-12", "G1 X8 Y-14", "G1 X6 Y-16", "G1 X4 Y-17", "G1 X2 Y-18",
    "G1 X0 Y-18", "G1 X-2 Y-18", "G1 X-4 Y-17", "G1 X-6 Y-16", "G1 X-8 Y-14", "G1 X-9 Y-12",
    "G1 X-10 Y-10", "G1 X-10 Y-8", "G1 X-9 Y-6", "G1 X-8 Y-4", "G1 X-6 Y-2", "G1 X-4 Y-1",
    "G1 X-2 Y0", "G1 X0 Y0", "M400", "M971",
]

MOVE45 = [
    "M972", "G1 X0 Y0 Z120 F6000",
    "G1 X5 Y0", "G1 X10 Y2", "G1 X15 Y5", "G1 X20 Y10", "G1 X23 Y15", "G1 X25 Y20",
    "G1 X23 Y25", "G1 X20 Y30", "G1 X15 Y35", "G1 X10 Y38", "G1 X5 Y40", "G1 X0 Y40",
    "G1 X-5 Y40", "G1 X-10 Y38", "G1 X-15 Y35", "G1 X-20 Y30", "G1 X-23 Y25", "G1 X-25 Y20",
    "G1 X-23 Y15", "G1 X-20 Y10", "G1 X-15 Y5", "G1 X-10 Y2", "G1 X-5 Y0", "G1 X0 Y0",
    "G1 X6 Y-2", "G1 X12 Y-5", "G1 X18 Y-10", "G1 X22 Y-16", "G1 X24 Y-22", "G1 X22 Y-28",
    "G1 X18 Y-34", "G1 X12 Y-38", "G1 X6 Y-40", "G1 X0 Y-40", "G1 X-6 Y-40", "G1 X-12 Y-38",
    "G1 X-18 Y-34", "G1 X-22 Y-28", "G1 X-24 Y-22", "G1 X-22 Y-16", "G1 X-18 Y-10", "G1 X-12 Y-5",
    "G1 X-6 Y-2", "G1 X0 Y0", "M400", "M971",
]

REVERSAL = [
    "M972", "G1 X0 Y0 Z120 F10800",
    "G1 X35 Y0", "G1 X-35 Y0", "G1 X35 Y0", "G1 X-35 Y0", "G1 X35 Y0",
    "G1 X0 Y0", "G1 X0 Y35", "G1 X0 Y-35", "G1 X0 Y35", "G1 X0 Y-35", "G1 X0 Y0",
    "M400", "M971",
]

DIAGONAL = ["M972", "G1 X40 Y0 Z120 F4800", "M400", "M971"]

PERF_RE = re.compile(
    r"PERF .*?blocks=(?P<blocks>\d+).*?vevents=(?P<vevents>\d+).*?steps=(?P<a>\d+)/(?P<b>\d+)/(?P<c>\d+)"
    r".*?starves=(?P<starves>\d+).*?guards=(?P<guards>\d+).*?phase_fault=(?P<phase_fault>\d+)"
    r".*?interval_ticks=(?P<imin>\d+)\.\.(?P<imax>\d+).*?health=(?P<health>[A-Z_]+)"
)

@dataclasses.dataclass
class Perf:
    blocks: int
    vevents: int
    a: int
    b: int
    c: int
    starves: int
    guards: int
    phase_fault: int
    imin: int
    imax: int
    health: str

@dataclasses.dataclass
class CaseResult:
    name: str
    passed: bool
    reason: str
    perf: Optional[Perf] = None

class Link:
    def __init__(self, port: str, log_path: Path):
        self.ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.05, write_timeout=2)
        self.log_path = log_path
        self.log = log_path.open("w", encoding="utf-8", buffering=1)
        self.lines: queue.Queue[str] = queue.Queue()
        self.stop = threading.Event()
        self.reader = threading.Thread(target=self._reader, daemon=True)
        self.reader.start()

    def _reader(self) -> None:
        while not self.stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                self.lines.put(f"__SERIAL_ERROR__ {exc}")
                return
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            stamp = time.monotonic()
            self.log.write(f"RX {stamp:.6f} {text}\n")
            print(f"< {text}")
            self.lines.put(text)

    def send(self, line: str) -> None:
        payload = (line.rstrip("\r\n") + "\n").encode("ascii")
        self.log.write(f"TX {time.monotonic():.6f} {line}\n")
        self.ser.write(payload)
        self.ser.flush()

    def send_raw(self, payload: bytes) -> None:
        self.log.write(f"TXRAW {time.monotonic():.6f} bytes={len(payload)}\n")
        self.ser.write(payload)
        self.ser.flush()

    def get(self, timeout: float) -> str:
        return self.lines.get(timeout=timeout)

    def drain(self) -> list[str]:
        out: list[str] = []
        while True:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                return out

    def close(self) -> None:
        self.stop.set()
        try:
            self.reader.join(timeout=0.2)
        finally:
            self.ser.close()
            self.log.close()


def autodetect_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found")
    likely = [p for p in ports if any(k in (p.description or "").lower() for k in ("arduino", "mega", "ch340", "usb serial"))]
    chosen = likely[0] if likely else ports[0]
    print("Detected serial ports:")
    for p in ports:
        marker = "  <-- selected" if p.device == chosen.device else ""
        print(f"  {p.device:12s} {p.description}{marker}")
    return chosen.device


def wait_for(link: Link, predicate, timeout: float, label: str) -> list[str]:
    deadline = time.monotonic() + timeout
    seen: list[str] = []
    while time.monotonic() < deadline:
        try:
            line = link.get(min(0.25, max(0.01, deadline - time.monotonic())))
        except queue.Empty:
            continue
        seen.append(line)
        if line.startswith("__SERIAL_ERROR__"):
            raise RuntimeError(line)
        if predicate(line):
            return seen
    raise TimeoutError(f"timeout waiting for {label}; last={seen[-8:]}")


def wait_ok(link: Link, timeout: float = 3.0) -> list[str]:
    return wait_for(link, lambda s: s == "ok" or s.startswith("ok "), timeout, "ok")


def command(link: Link, cmd: str, timeout: float = 3.0) -> list[str]:
    link.send(cmd)
    return wait_ok(link, timeout)


def parse_perf(lines: Iterable[str]) -> Optional[Perf]:
    last = None
    for line in lines:
        m = PERF_RE.search(line)
        if not m:
            continue
        g = m.groupdict()
        last = Perf(
            blocks=int(g["blocks"]), vevents=int(g["vevents"]),
            a=int(g["a"]), b=int(g["b"]), c=int(g["c"]),
            starves=int(g["starves"]), guards=int(g["guards"]), phase_fault=int(g["phase_fault"]),
            imin=int(g["imin"]), imax=int(g["imax"]), health=g["health"],
        )
    return last


def assert_clean(perf: Perf) -> Optional[str]:
    if perf.starves != 0: return f"starves={perf.starves}"
    if perf.guards != 0: return f"guards={perf.guards}"
    if perf.phase_fault != 0: return f"phase_fault={perf.phase_fault}"
    if perf.health != "CLEAN": return f"health={perf.health}"
    return None


def setup_diagnostics(link: Link) -> bool:
    print("\n=== SETUP DIAGNOSTICS ===")
    command(link, "M119")
    print("Verify all three MAX endstops report open before homing.")
    answer = input("Type YES when motors are mechanically safe and switches can be reached: ").strip().upper()
    if answer != "YES":
        print("Setup aborted without motion.")
        return False
    link.send("G28")
    lines = wait_for(link, lambda s: s == "ok", 30.0, "G28 completion")
    joined = "\n".join(lines)
    ok = "HOME_DONE" in joined and "A:TRIGGERED" in joined and "B:TRIGGERED" in joined and "C:TRIGGERED" in joined
    print("SETUP HOMING:", "PASS" if ok else "FAIL")
    command(link, "M114")
    return ok


def ensure_homed(link: Link) -> None:
    link.drain()
    link.send("G28")
    lines = wait_for(link, lambda s: s == "ok", 30.0, "G28")
    text = "\n".join(lines)
    if "HOME_DONE" not in text or "HOMED:YES" not in text:
        raise RuntimeError("G28 did not produce a valid HOME_DONE/HOMED:YES state")


def run_ack_case(link: Link, name: str, commands: list[str], expected: dict) -> CaseResult:
    link.drain()
    all_lines: list[str] = []
    try:
        for cmd in commands:
            link.send(cmd)
            if cmd == "M400":
                all_lines += wait_for(link, lambda s: s == "ok", 30.0, "M400")
            else:
                all_lines += wait_ok(link, 5.0)
        all_lines += link.drain()
    except Exception as exc:
        return CaseResult(name, False, f"transport/timeout: {exc}")

    bad = [s for s in all_lines if "UNKNOWN_COMMAND" in s or "MALFORMED_COMMAND" in s or "LINE_TOO_LONG" in s or "QUEUE_FULL" in s or "FAULT" in s]
    if bad:
        return CaseResult(name, False, f"firmware error: {bad[-1]}")
    perf = parse_perf(all_lines)
    if not perf:
        return CaseResult(name, False, "no PERF line")
    dirty = assert_clean(perf)
    if dirty:
        return CaseResult(name, False, dirty, perf)
    for key, value in expected.items():
        if getattr(perf, key) != value:
            return CaseResult(name, False, f"{key}={getattr(perf,key)} expected={value}", perf)
    return CaseResult(name, True, "golden counts match", perf)


def run_raw_burst_with_m105(link: Link, rounds: int = 10) -> CaseResult:
    """Send a real unpaced 45-move paste while a second writer injects M105.

    This intentionally reproduces the failure mode seen on hardware. It is a
    transport/scheduler stress test, not a recommended host protocol.
    """
    link.drain()
    failures: list[str] = []
    for r in range(rounds):
        command(link, "M972")
        payload = ("\n".join(MOVE45[1:-2]) + "\nM400\nM971\n").encode("ascii")
        done = threading.Event()

        def poller() -> None:
            while not done.is_set():
                time.sleep(0.015)
                try:
                    link.send("M105")
                except Exception:
                    return

        t = threading.Thread(target=poller, daemon=True)
        t.start()
        try:
            link.send_raw(payload)
            lines = wait_for(link, lambda s: "PERF " in s and "health=" in s, 45.0, "burst PERF")
            # capture trailing ACKs/errors after PERF
            time.sleep(0.05)
            lines += link.drain()
        except Exception as exc:
            failures.append(f"round {r+1}: {exc}")
            done.set(); t.join(timeout=0.2)
            break
        done.set(); t.join(timeout=0.2)
        bad = [s for s in lines if any(x in s for x in ("UNKNOWN_COMMAND", "MALFORMED_COMMAND", "LINE_TOO_LONG", "QUEUE_FULL", "FAULT"))]
        if bad:
            failures.append(f"round {r+1}: {bad[-1]}")
            break
        perf = parse_perf(lines)
        if not perf:
            failures.append(f"round {r+1}: no PERF")
            break
        dirty = assert_clean(perf)
        if dirty:
            failures.append(f"round {r+1}: {dirty}")
            break
    if failures:
        return CaseResult("RAW45+M105", False, failures[-1])
    return CaseResult("RAW45+M105", True, f"{rounds}/{rounds} real USB/UART rounds clean")


def firmware_sanity(link: Link) -> CaseResult:
    link.drain()
    try:
        lines = command(link, "M115")
        text = "\n".join(lines)
        if "FIRMWARE_NAME:DeltaCore" not in text:
            return CaseResult("FIRMWARE", False, "M115 is not DeltaCore")
        command(link, "M111 S2")
        command(link, "M503")
        return CaseResult("FIRMWARE", True, "M115/M503 responsive")
    except Exception as exc:
        return CaseResult("FIRMWARE", False, str(exc))


def print_result(r: CaseResult) -> None:
    mark = "PASS" if r.passed else "FAIL"
    extra = ""
    if r.perf:
        extra = f" | blocks={r.perf.blocks} vevents={r.perf.vevents} steps={r.perf.a}/{r.perf.b}/{r.perf.c} health={r.perf.health}"
    print(f"[{mark:4s}] {r.name:18s} {r.reason}{extra}")


def main() -> int:
    ap = argparse.ArgumentParser(description="DeltaCore real Mega2560/RAMPS HIL runner")
    ap.add_argument("--port", help="COM port, e.g. COM5; auto-detect if omitted")
    ap.add_argument("--setup", action="store_true", help="interactive endstop/homing setup only")
    ap.add_argument("--rounds", type=int, default=10, help="RAW45+M105 stress rounds")
    ap.add_argument("--no-raw", action="store_true", help="skip dangerous unpaced burst stress")
    ap.add_argument("--log-dir", default="real_hil_logs")
    args = ap.parse_args()

    port = args.port or autodetect_port()
    stamp = time.strftime("%Y%m%d-%H%M%S")
    log_dir = Path(args.log_dir); log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"deltacore-real-hil-{stamp}.log"
    report_path = log_dir / f"deltacore-real-hil-{stamp}.json"

    print(f"Opening {port} @ {BAUD} baud")
    link = Link(port, log_path)
    results: list[CaseResult] = []
    try:
        # DTR reset can take a moment; accept either the boot banner or an already-running board.
        time.sleep(2.2)
        link.drain()
        if args.setup:
            return 0 if setup_diagnostics(link) else 1

        r = firmware_sanity(link); results.append(r); print_result(r)
        if not r.passed: return 1
        ensure_homed(link)

        cases = [
            ("DIAGONAL", DIAGONAL, {"blocks":154, "vevents":10154, "a":10153, "b":7452, "c":8741}),
            ("SHORT_F7200", SHORT_SEGMENT, {"blocks":192, "vevents":4358, "a":2892, "b":2892, "c":2756}),
            ("REVERSAL_F10800", REVERSAL, {"blocks":661, "vevents":19246, "a":12620, "b":12620, "c":10780}),
        ]
        for name, cmds, expected in cases:
            r = run_ack_case(link, name, cmds, expected)
            results.append(r); print_result(r)
            if not r.passed:
                break

        if all(r.passed for r in results) and not args.no_raw:
            r = run_raw_burst_with_m105(link, max(1, args.rounds))
            results.append(r); print_result(r)

    finally:
        link.close()
        report = {
            "port": port, "baud": BAUD, "log": str(log_path),
            "results": [dataclasses.asdict(r) for r in results],
            "passed": bool(results) and all(r.passed for r in results),
        }
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nLog:    {log_path}")
        print(f"Report: {report_path}")

    passed = bool(results) and all(r.passed for r in results)
    print("\nREAL-HIL RESULT:", "PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
