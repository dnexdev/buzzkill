#pragma once

#include "servo.h"

#include <string>

namespace buzzkill {

// Drives the ESP32 turret firmware over a POSIX serial device.
// Protocol is documented in arduino/buzzkill/buzzkill.ino.
class SerialServo : public Servo {
public:
    // dev: path like "/dev/serUSB0" (QNX) or "/dev/ttyUSB0" / "/dev/tty.usbserial-*" (Linux/mac)
    // baud: match the ESP32 sketch — 115200
    SerialServo(const std::string& dev, int baud = 115200);
    ~SerialServo() override;

    bool ok() const { return fd_ >= 0; }

    void aim(float pan_deg, float tilt_deg) override;
    void fire(int duration_ms) override;
    void disarm() override;

    // Call periodically (e.g. every 100ms) so the ESP32 watchdog doesn't kick in.
    void heartbeat();

    // Spin flywheels on/off. Tracker calls this on target acquire/lose so the
    // wheels are already at speed by the time we fire.
    void set_spin(bool on);

private:
    int  fd_ = -1;
    bool spin_ = false;
    float last_pan_ = 1e9f, last_tilt_ = 1e9f;
    void write_line(const char* s);
};

} // namespace buzzkill
