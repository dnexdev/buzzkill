"""Buzzkill controller — runs on the RP5/QNX box.

Reads target packets over UDP from detect.py and drives the turret over
serial using arrow.sh's PAN:/TILT: command set. Runs a proportional (P-only)
controller per axis on the pixel error between the detected target and the
frame center, capped at --max-step degrees per tick.

Usage:
  python3 main.py --serial /dev/ser1 --port 9000
  python3 main.py --dry-run --port 9000   # print commands, no serial
"""
from __future__ import annotations

import argparse
import signal
import time

from receiver import UdpReceiver


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SerialSink:
    def __init__(self, dev: str, baud: int, wait_boot: float):
        import serial  # pyserial

        self._port = serial.Serial(dev, baud, timeout=0, write_timeout=0.5)
        time.sleep(wait_boot)  # let the board finish any reset-on-open
        try:
            self._port.reset_input_buffer()
        except Exception:
            pass

    def send(self, line: str) -> None:
        self._port.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        try:
            self._port.close()
        except Exception:
            pass


class PrintSink:
    def send(self, line: str) -> None:
        print(f"[main] > {line}", flush=True)

    def close(self) -> None:
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9000,
                    help="UDP port detect.py sends target packets to")
    ap.add_argument("--serial", default="/dev/ser1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true",
                    help="print commands instead of writing to serial")
    ap.add_argument("--kp-pan", type=float, default=0.08, help="pan gain, deg/px")
    ap.add_argument("--kp-tilt", type=float, default=0.08, help="tilt gain, deg/px")
    ap.add_argument("--max-step", type=float, default=20.0,
                    help="max degrees per control tick (speed cap)")
    ap.add_argument("--hz", type=float, default=30.0, help="control loop rate")
    ap.add_argument("--wait-boot", type=float, default=2.0)
    args = ap.parse_args()

    stop = [False]
    def on_sigint(*_):
        stop[0] = True
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    sink = PrintSink() if args.dry_run else SerialSink(args.serial, args.baud, args.wait_boot)
    rx = UdpReceiver(args.port)
    print(f"[main] listening on udp:{args.port}", flush=True)

    had_target = False
    tick_dt = 1.0 / args.hz
    next_tick = time.monotonic()

    try:
        while not stop[0]:
            pkt = rx.poll()

            if pkt is not None and pkt.get("det"):
                if not had_target:
                    print("[main] target acquired", flush=True)
                    had_target = True

                fw = max(1, int(pkt.get("fw", 640)))
                fh = max(1, int(pkt.get("fh", 480)))
                cx, cy = fw / 2.0, fh / 2.0
                ex = float(pkt["x"]) - cx
                ey = float(pkt["y"]) - cy

                # P-only per axis: output proportional to error, capped at max_step.
                pan_step = clamp(args.kp_pan * ex, -args.max_step, args.max_step)
                tilt_step = clamp(args.kp_tilt * ey, -args.max_step, args.max_step)

                moved = []
                if abs(pan_step) >= 1:
                    sink.send(f"PAN:{int(round(pan_step))}")
                    moved.append(f"PAN:{int(round(pan_step))}")
                if abs(tilt_step) >= 1:
                    sink.send(f"TILT:{int(round(tilt_step))}")
                    moved.append(f"TILT:{int(round(tilt_step))}")

                if moved:
                    print(f"[main] move sent: {' '.join(moved)}  "
                          f"(ex={ex:+.1f} ey={ey:+.1f})", flush=True)
                else:
                    print(f"[main] no move — within deadband "
                          f"(ex={ex:+.1f} ey={ey:+.1f})", flush=True)
            elif had_target:
                print("[main] target lost", flush=True)
                had_target = False

            next_tick += tick_dt
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()  # fell behind, resync
    finally:
        sink.close()
        rx.close()
        print("\n[main] bye", flush=True)


if __name__ == "__main__":
    main()
