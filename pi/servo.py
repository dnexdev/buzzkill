"""Servo backends. All talk to the ESP32 sketch protocol.

Sketch protocol (line-delimited, \\n terminated, 115200 baud):
  E                 arm
  D                 disarm
  A <pan> <tilt>    aim; integer degrees offset from center, ±60
  F                 flywheels ON
  f                 flywheels OFF
  P                 fire one dart (armed only)
  S                 emergency stop
"""
from __future__ import annotations

import sys
import time


class Servo:
    def aim(self, pan_deg: float, tilt_deg: float) -> None: ...
    def fire(self) -> None: ...
    def spin(self, on: bool) -> None: ...
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def close(self) -> None: ...


class MockServo(Servo):
    def __init__(self) -> None:
        self._last = (1e9, 1e9)
        self._spin = False

    def aim(self, pan_deg, tilt_deg):
        if abs(pan_deg - self._last[0]) < 0.25 and abs(tilt_deg - self._last[1]) < 0.25:
            return
        self._last = (pan_deg, tilt_deg)
        print(f"[servo] aim  pan={pan_deg:+6.2f}  tilt={tilt_deg:+6.2f}", flush=True)

    def fire(self):
        print("[servo] FIRE", flush=True)

    def spin(self, on):
        if on == self._spin:
            return
        self._spin = on
        print(f"[servo] spin {'ON' if on else 'OFF'}", flush=True)

    def arm(self):    print("[servo] ARM", flush=True)
    def disarm(self): print("[servo] DISARM", flush=True)
    def close(self):  self.disarm()


class SerialServo(Servo):
    def __init__(self, dev: str, baud: int = 115200, wait_boot: float = 1.5):
        import serial  # pyserial

        # Timeout=0 = non-blocking reads. write_timeout stops us hanging on a
        # dead port. ESP32 resets when the port is opened, so wait_boot lets
        # the sketch finish `setup()` before we start shoving commands in.
        self._port = serial.Serial(
            dev, baud, timeout=0, write_timeout=0.5,
        )
        time.sleep(wait_boot)
        # Drain the "Buzzkill turret ready" boot banner.
        try:
            self._port.reset_input_buffer()
        except Exception:
            pass

        self._last = (1e9, 1e9)
        self._spin = False
        self._armed = False

    def _send(self, line: str) -> None:
        try:
            self._port.write((line + "\n").encode("ascii"))
        except Exception as e:
            print(f"[servo] serial write failed: {e}", file=sys.stderr)

    def _drain(self) -> None:
        # Read and discard any pending reply lines so the buffer never fills.
        try:
            self._port.read(4096)
        except Exception:
            pass

    def aim(self, pan_deg, tilt_deg):
        if abs(pan_deg - self._last[0]) < 0.25 and abs(tilt_deg - self._last[1]) < 0.25:
            return
        self._last = (pan_deg, tilt_deg)
        # Sketch expects integer offsets from center; clamp to sketch limits (±60).
        p = max(-60, min(60, int(round(pan_deg))))
        t = max(-45, min(45, int(round(tilt_deg))))
        self._send(f"A {p} {t}")
        if not self._spin:
            self._send("F")
            self._spin = True
        self._drain()

    def fire(self):
        self._send("P")
        self._drain()

    def spin(self, on):
        if on == self._spin:
            return
        self._spin = on
        self._send("F" if on else "f")
        self._drain()

    def arm(self):
        if self._armed:
            return
        self._send("E")
        self._armed = True
        self._drain()

    def disarm(self):
        self._send("D")
        self._spin = False
        self._armed = False
        self._last = (1e9, 1e9)
        self._drain()

    def close(self):
        try:
            self.disarm()
        finally:
            try:
                self._port.close()
            except Exception:
                pass
