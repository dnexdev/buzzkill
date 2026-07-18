"""4-corner bilinear pixel → (pan, tilt) mapping."""
from __future__ import annotations

import json


CORNERS = ("top-left", "top-right", "bottom-right", "bottom-left")


class Calibration:
    def __init__(self):
        self.frame_w = 640
        self.frame_h = 480
        # Default: linear ±30° span so we do something sane before calibration.
        self._corners = {
            "top-left":     (-30.0,  20.0),
            "top-right":    ( 30.0,  20.0),
            "bottom-right": ( 30.0, -20.0),
            "bottom-left":  (-30.0, -20.0),
        }

    @classmethod
    def load(cls, path: str) -> "Calibration":
        c = cls()
        with open(path) as f:
            data = json.load(f)
        c.frame_w = int(data.get("frame_w", 640))
        c.frame_h = int(data.get("frame_h", 480))
        for pt in data.get("points", []):
            name = pt["name"]
            if name in c._corners:
                c._corners[name] = (float(pt["pan"]), float(pt["tilt"]))
        return c

    def pixel_to_angles(self, px: float, py: float) -> tuple[float, float]:
        w = max(1, self.frame_w)
        h = max(1, self.frame_h)
        u = max(0.0, min(1.0, px / w))
        v = max(0.0, min(1.0, py / h))
        tl = self._corners["top-left"]
        tr = self._corners["top-right"]
        br = self._corners["bottom-right"]
        bl = self._corners["bottom-left"]

        def blend(a, b, c, d):
            top = a * (1 - u) + b * u
            bot = d * (1 - u) + c * u
            return top * (1 - v) + bot * v

        pan  = blend(tl[0], tr[0], br[0], bl[0])
        tilt = blend(tl[1], tr[1], br[1], bl[1])
        return pan, tilt
