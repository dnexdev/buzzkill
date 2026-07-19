"""Servo backends. Talk to the ESP32 sketch that lives in arduino/buzzkill.

Sketch protocol (line-delimited, \\n terminated, 115200 baud):
  PAN:<deg>,TILT:<deg>            aim only — absolute degrees, 0-180 pan, 60-120 tilt
  PAN:<deg>,TILT:<deg>,FIRE       aim AND fire (ESC spins for 1.5 s, sketch handles timing)
  IDLE                            hold last position (no-op)

Notes:
  - Angles are absolute (center=90), NOT offsets from center. The main.py
    tracker still emits offsets from center; this class converts.
  - There is no separate arm/disarm/spin — the ESC is armed at sketch boot
    and only spins during a FIRE cycle. arm()/disarm()/spin() are no-ops
    on the wire but tracked internally so main.py's callers still work.
  - The sketch reads from Serial2 (UART pins), so on the Pi you wire:
      Pi TX  -> ESP32 GPIO 13 (RXD2)
      Pi RX  -> ESP32 GPIO 14 (TXD2)
      Pi GND -> ESP32 GND
    and open the Pi's UART device (e.g. /dev/ser1 on QNX, /dev/serial0 on
    Raspberry Pi OS), NOT the USB-serial device that Arduino IDE uses.
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
    # Sketch's mechanical limits — must match the ESP32 firmware constants
    # (PAN_MIN/MAX and TILT_MIN/MAX) or aim commands get silently clipped.
    PAN_ABS_MIN, PAN_ABS_MAX, PAN_CENTER = 0, 180, 90
    TILT_ABS_MIN, TILT_ABS_MAX, TILT_CENTER = 60, 120, 90

    def __init__(self, dev: str, baud: int = 115200, wait_boot: float = 3.0):
        import serial  # pyserial

        # timeout=0 is non-blocking; write_timeout guards against a dead port.
        # wait_boot must exceed the ESC arm delay in the sketch (2 s) so we
        # don't shove commands at it mid-arm.
        self._port = serial.Serial(
            dev, baud, timeout=0, write_timeout=0.5,
        )
        time.sleep(wait_boot)
        try:
            self._port.reset_input_buffer()
        except Exception:
            pass

        # Absolute angles most recently commanded. FIRE messages need to
        # include current aim so the sketch doesn't reset servos to center.
        self._last_pan_abs = self.PAN_CENTER
        self._last_tilt_abs = self.TILT_CENTER
        # Dedup guard so we don't spam identical PAN:x,TILT:y lines.
        self._last_sent = (-999, -999)

    def _send(self, line: str) -> None:
        try:
            self._port.write((line + "\n").encode("ascii"))
        except Exception as e:
            print(f"[servo] serial write failed: {e}", file=sys.stderr)

    def _drain(self) -> None:
        try:
            self._port.read(4096)
        except Exception:
            pass

    def _to_abs(self, pan_offset: float, tilt_offset: float) -> tuple[int, int]:
        pan_abs  = int(round(self.PAN_CENTER  + pan_offset))
        tilt_abs = int(round(self.TILT_CENTER + tilt_offset))
        pan_abs  = max(self.PAN_ABS_MIN,  min(self.PAN_ABS_MAX,  pan_abs))
        tilt_abs = max(self.TILT_ABS_MIN, min(self.TILT_ABS_MAX, tilt_abs))
        return pan_abs, tilt_abs

    def aim(self, pan_deg, tilt_deg):
        pan_abs, tilt_abs = self._to_abs(pan_deg, tilt_deg)
        # Skip repeats — the sketch already clamps and echoes over Serial.
        if pan_abs == self._last_sent[0] and tilt_abs == self._last_sent[1]:
            return
        self._last_sent    = (pan_abs, tilt_abs)
        self._last_pan_abs = pan_abs
        self._last_tilt_abs = tilt_abs
        self._send(f"PAN:{pan_abs},TILT:{tilt_abs}")
        self._drain()

    def fire(self):
        # Include current aim so this same line both aims and fires — matches
        # the sketch's expected `PAN:...,TILT:...,FIRE` format exactly.
        self._send(
            f"PAN:{self._last_pan_abs},TILT:{self._last_tilt_abs},FIRE"
        )
        self._drain()

    def spin(self, on):
        # The sketch controls the ESC internally as part of the FIRE cycle.
        # No wire-level spin toggle; keep this as a no-op so callers don't
        # need to branch on backend.
        pass

    def arm(self):
        # ESC is armed at sketch boot. No wire message needed.
        pass

    def disarm(self):
        # Nothing to disarm on the sketch side — send IDLE for symmetry
        # (holds last position) and reset our dedup state.
        self._send("IDLE")
        self._last_sent = (-999, -999)
        self._drain()

    def close(self):
        try:
            self.disarm()
        finally:
            try:
                self._port.close()
            except Exception:
                pass
