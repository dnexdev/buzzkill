#pragma once

#include "servo.h"

namespace buzzkill {

class MockServo : public Servo {
public:
    void aim(float pan_deg, float tilt_deg) override;
    void fire(int duration_ms) override;
    void disarm() override;
private:
    float last_pan_ = 0;
    float last_tilt_ = 0;
};

} // namespace buzzkill
