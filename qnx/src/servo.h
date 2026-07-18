#pragma once

namespace buzzkill {

// Servo abstraction. Implementations: mock (prints), pca9685 (I2C).
class Servo {
public:
    virtual ~Servo() = default;
    // angles in degrees; pan positive = right, tilt positive = up
    virtual void aim(float pan_deg, float tilt_deg) = 0;
    // engages flywheel + pusher for a burst of the given duration (ms).
    virtual void fire(int duration_ms) = 0;
    // safe idle state
    virtual void disarm() = 0;
};

} // namespace buzzkill
