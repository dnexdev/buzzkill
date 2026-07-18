#pragma once

#include "protocol.h"

namespace buzzkill {

struct TrackerConfig {
    float ema_alpha         = 0.4f;   // position smoothing
    float lead_seconds      = 0.12f;  // servo + bullet travel; tune on-site
    float stale_seconds     = 0.20f;  // drop target if older than this
    int   min_hits_to_fire  = 4;      // consecutive detections needed
    float fire_tolerance_px = 30.f;   // predicted vs current aim
    float fire_cooldown_s   = 0.35f;  // between bursts
    int   fire_burst_ms     = 120;
    float min_confidence    = 0.15f;
};

// Fed a stream of Target packets, produces a smoothed & lead-predicted aim
// point and decides when to fire.
class Tracker {
public:
    explicit Tracker(TrackerConfig cfg = {}) : cfg_(cfg) {}

    // Ingest a fresh packet. `now` is monotonic seconds.
    void update(const Target& t, double now);

    // Aim point to feed the servos. Returns false if no valid target.
    bool aim_point(double now, float& px, float& py) const;

    // Should we fire this tick? Consumes cooldown when returning true.
    bool should_fire(double now);

    int  fire_burst_ms() const { return cfg_.fire_burst_ms; }

private:
    TrackerConfig cfg_;
    bool  have_ = false;
    float sx_ = 0, sy_ = 0;   // smoothed position
    float vx_ = 0, vy_ = 0;   // smoothed velocity
    double last_update_ = 0;
    double last_fire_ = -1e9;
    int   hits_ = 0;
    float last_conf_ = 0;
};

} // namespace buzzkill
