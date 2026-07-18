// Wire format shared with laptop/protocol.py. Keep in sync.
// JSON over UDP. All fields required.
//
// { "v": 1, "t": 1234567890.123, "fw": 640, "fh": 480,
//   "x": 320.0, "y": 240.0, "vx": 12.3, "vy": -4.5,
//   "conf": 0.8, "det": true }
#pragma once

#include <cstdint>
#include <string>

namespace buzzkill {

struct Target {
    double   t_recv;   // local receive time, seconds
    double   t_send;   // sender timestamp
    uint16_t frame_w;
    uint16_t frame_h;
    float    x, y;     // pixel centroid; -1 if not detected
    float    vx, vy;   // pixels/sec
    float    confidence;
    bool     detected;
};

// Minimal JSON decoder for our packet. Returns true on success.
bool parse_target(const char* buf, size_t len, Target& out);

} // namespace buzzkill
