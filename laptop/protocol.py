"""Wire format shared with qnx/src/protocol.h. Keep in sync."""
import json
import time


def make_packet(frame_w, frame_h, target_x, target_y, vx, vy, confidence, detected):
    return {
        "v": 1,
        "t": time.time(),
        "fw": int(frame_w),
        "fh": int(frame_h),
        "x": float(target_x),
        "y": float(target_y),
        "vx": float(vx),
        "vy": float(vy),
        "conf": float(confidence),
        "det": bool(detected),
    }


def encode(pkt):
    return json.dumps(pkt, separators=(",", ":")).encode("utf-8")
