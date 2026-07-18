"""Buzzkill controller — runs on the RP5/QNX box.

Reads target packets over UDP from detect.py, smooths them, predicts lead,
maps pixels to servo angles, and drives the ESP32 turret over serial.

Usage:
  python3 main.py --mock --port 9000
  python3 main.py --serial /dev/serUSB0 --calib config/calibration.json
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

from calibration import Calibration
from receiver import UdpReceiver
from servo import MockServo, SerialServo
from tracker import Tracker


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mock", action="store_true", help="print servo commands instead of talking to ESP32")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--serial", default="/dev/serUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--calib", default="config/calibration.json")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    stop = [False]
    def on_sigint(*_):
        stop[0] = True
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    rx = UdpReceiver(args.port)
    print(f"[main] listening on udp:{args.port}", flush=True)

    try:
        calib = Calibration.load(args.calib)
        print(f"[main] loaded {args.calib} ({calib.frame_w}x{calib.frame_h})", flush=True)
    except Exception as e:
        print(f"[main] no calibration: {e}. using default ±30° span.", flush=True)
        calib = Calibration()

    if args.mock:
        servo = MockServo()
        print("[main] servo: mock", flush=True)
    else:
        try:
            servo = SerialServo(args.serial, args.baud)
            print(f"[main] servo: esp32 on {args.serial}", flush=True)
        except Exception as e:
            print(f"[main] serial open failed ({e}); falling back to mock", flush=True)
            servo = MockServo()
    servo.arm()

    tracker = Tracker()

    tick_dt = 1.0 / args.hz
    next_tick = time.monotonic()
    last_pkt_t = 0.0
    have_target_at_all = False
    stats = {"pkts": 0, "fires": 0}
    next_log = time.monotonic() + 1.0

    try:
        while not stop[0]:
            pkt = rx.poll()
            now = time.monotonic()
            if pkt is not None:
                stats["pkts"] += 1
                last_pkt_t = now
                tracker.update(pkt, now)
                have_target_at_all = True

            aim = tracker.aim_point(now)
            if aim is not None:
                px, py = aim
                pan, tilt = calib.pixel_to_angles(px, py)
                servo.aim(pan, tilt)
                if tracker.should_fire(now):
                    servo.fire()
                    stats["fires"] += 1
            elif have_target_at_all and now - last_pkt_t > 1.0:
                # No packets for a full second — stop flywheels, hold position.
                servo.spin(False)
                have_target_at_all = False

            if args.verbose and now >= next_log:
                print(f"[main] pkts={stats['pkts']}/s fires={stats['fires']}", flush=True)
                stats["pkts"] = stats["fires"] = 0
                next_log = now + 1.0

            next_tick += tick_dt
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.monotonic()  # fell behind, resync
    finally:
        servo.close()
        rx.close()
        print("\n[main] bye", flush=True)


if __name__ == "__main__":
    main()
