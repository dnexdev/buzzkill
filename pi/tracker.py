"""EMA smoothing + linear lead prediction + fire decision.

Ported from qnx/src/tracker.{h,cpp}. Same math, same defaults.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class TrackerConfig:
    ema_alpha: float = 0.4          # position/velocity smoothing
    lead_seconds: float = 0.12      # servo + bullet travel; tune on-site
    stale_seconds: float = 0.20     # drop target if no update within
    min_hits_to_fire: int = 4       # consecutive detections
    fire_tolerance_px: float = 30.0
    fire_cooldown_s: float = 10.0
    min_confidence: float = 0.15


class Tracker:
    def __init__(self, cfg: TrackerConfig | None = None):
        self.cfg = cfg or TrackerConfig()
        self._have = False
        self._sx = self._sy = 0.0
        self._vx = self._vy = 0.0
        self._last_update = 0.0
        self._last_fire = -1e9
        self._hits = 0

    def update(self, pkt: dict, now: float) -> None:
        if not pkt.get("det") or pkt.get("conf", 0) < self.cfg.min_confidence:
            self._hits = 0
            return
        a = self.cfg.ema_alpha
        x  = float(pkt["x"]);  y  = float(pkt["y"])
        vx = float(pkt.get("vx", 0.0)); vy = float(pkt.get("vy", 0.0))
        if not self._have:
            self._sx, self._sy = x, y
            self._vx, self._vy = vx, vy
            self._have = True
        else:
            self._sx = a * x + (1 - a) * self._sx
            self._sy = a * y + (1 - a) * self._sy
            self._vx = a * vx + (1 - a) * self._vx
            self._vy = a * vy + (1 - a) * self._vy
        self._last_update = now
        if self._hits < 10000:
            self._hits += 1

    def aim_point(self, now: float):
        """Return (px, py) predicted aim, or None if no valid target."""
        if not self._have:
            return None
        if now - self._last_update > self.cfg.stale_seconds:
            return None
        return (
            self._sx + self._vx * self.cfg.lead_seconds,
            self._sy + self._vy * self.cfg.lead_seconds,
        )

    def should_fire(self, now: float) -> bool:
        if not self._have:
            return False
        if now - self._last_update > self.cfg.stale_seconds:
            return False
        if self._hits < self.cfg.min_hits_to_fire:
            return False
        if now - self._last_fire < self.cfg.fire_cooldown_s:
            return False

        dx = self._vx * self.cfg.lead_seconds
        dy = self._vy * self.cfg.lead_seconds
        lead_dist = math.hypot(dx, dy)
        # If predicted lead is huge, target is too fast — hold this tick.
        if lead_dist > 4 * self.cfg.fire_tolerance_px:
            return False

        self._last_fire = now
        return True
