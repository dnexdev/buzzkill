#include "tracker.h"

#include <cmath>

namespace buzzkill {

void Tracker::update(const Target& t, double now) {
    if (!t.detected || t.confidence < cfg_.min_confidence) {
        hits_ = 0;
        return;
    }
    float a = cfg_.ema_alpha;
    if (!have_) {
        sx_ = t.x; sy_ = t.y;
        vx_ = t.vx; vy_ = t.vy;
        have_ = true;
    } else {
        sx_ = a * t.x + (1 - a) * sx_;
        sy_ = a * t.y + (1 - a) * sy_;
        vx_ = a * t.vx + (1 - a) * vx_;
        vy_ = a * t.vy + (1 - a) * vy_;
    }
    last_update_ = now;
    last_conf_ = t.confidence;
    if (hits_ < 10000) ++hits_;
}

bool Tracker::aim_point(double now, float& px, float& py) const {
    if (!have_) return false;
    if (now - last_update_ > cfg_.stale_seconds) return false;
    px = sx_ + vx_ * cfg_.lead_seconds;
    py = sy_ + vy_ * cfg_.lead_seconds;
    return true;
}

bool Tracker::should_fire(double now) {
    if (!have_) return false;
    if (now - last_update_ > cfg_.stale_seconds) return false;
    if (hits_ < cfg_.min_hits_to_fire) return false;
    if (now - last_fire_ < cfg_.fire_cooldown_s) return false;

    float px = sx_ + vx_ * cfg_.lead_seconds;
    float py = sy_ + vy_ * cfg_.lead_seconds;
    float dx = px - sx_, dy = py - sy_;
    float lead_dist = std::sqrt(dx*dx + dy*dy);
    // If predicted lead is huge, target is fast — hold off a tick.
    if (lead_dist > 4 * cfg_.fire_tolerance_px) return false;

    last_fire_ = now;
    return true;
}

} // namespace buzzkill
