#include "servo_mock.h"

#include <cmath>
#include <cstdio>

namespace buzzkill {

void MockServo::aim(float pan_deg, float tilt_deg) {
    if (std::fabs(pan_deg - last_pan_) < 0.25f &&
        std::fabs(tilt_deg - last_tilt_) < 0.25f) return;
    last_pan_ = pan_deg;
    last_tilt_ = tilt_deg;
    std::printf("[servo] aim  pan=%+6.2f  tilt=%+6.2f\n", pan_deg, tilt_deg);
    std::fflush(stdout);
}

void MockServo::fire(int duration_ms) {
    std::printf("[servo] FIRE %d ms\n", duration_ms);
    std::fflush(stdout);
}

void MockServo::disarm() {
    std::printf("[servo] disarm\n");
    std::fflush(stdout);
}

} // namespace buzzkill
