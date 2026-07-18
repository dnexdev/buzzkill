#include "calibration.h"
#include "protocol.h"
#include "receiver.h"
#include "servo_mock.h"
#include "servo_serial.h"
#include "tracker.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <string>
#include <sys/time.h>
#include <time.h>

using namespace buzzkill;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

double now_seconds() {
    timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}

void sleep_seconds(double s) {
    timespec ts;
    ts.tv_sec  = static_cast<time_t>(s);
    ts.tv_nsec = static_cast<long>((s - ts.tv_sec) * 1e9);
    nanosleep(&ts, nullptr);
}
} // namespace

int main(int argc, char** argv) {
    bool mock = false;
    int port = 9000;
    std::string calib_path = "config/calibration.json";
    std::string serial_dev = "/dev/serUSB0";
    int serial_baud = 115200;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--mock") mock = true;
        else if (a == "--port"   && i + 1 < argc) port = std::atoi(argv[++i]);
        else if (a == "--calib"  && i + 1 < argc) calib_path = argv[++i];
        else if (a == "--serial" && i + 1 < argc) serial_dev = argv[++i];
        else if (a == "--baud"   && i + 1 < argc) serial_baud = std::atoi(argv[++i]);
        else if (a == "--help" || a == "-h") {
            std::printf(
              "usage: buzzkill [--mock] [--port N] [--calib path]\n"
              "                [--serial /dev/serUSB0] [--baud 115200]\n");
            return 0;
        } else {
            std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
            return 2;
        }
    }

    std::signal(SIGINT, on_sigint);

    Receiver rx;
    if (!rx.bind(port)) return 1;
    std::printf("[main] listening on udp:%d\n", port);

    Calibration calib;
    if (!calib.load(calib_path)) {
        std::printf("[main] no calib file, using default 640x480 ±30°\n");
        calib.set_default(640, 480, 30.f);
    }

    std::unique_ptr<Servo> servo;
    SerialServo* serial_ptr = nullptr;
    if (mock) {
        servo.reset(new MockServo());
        std::printf("[main] servo: mock\n");
    } else {
        auto* s = new SerialServo(serial_dev, serial_baud);
        if (!s->ok()) {
            std::fprintf(stderr, "[main] serial open failed, falling back to mock\n");
            delete s;
            servo.reset(new MockServo());
        } else {
            serial_ptr = s;
            servo.reset(s);
            std::printf("[main] servo: esp32 over %s\n", serial_dev.c_str());
        }
    }

    Tracker tracker;

    const double tick_hz = 50.0;
    const double tick_dt = 1.0 / tick_hz;

    Target last{};
    bool have_last = false;
    double next_tick = now_seconds();
    double next_heartbeat = now_seconds();

    while (!g_stop.load()) {
        Target t;
        if (rx.poll(t)) {
            last = t;
            have_last = true;
            tracker.update(t, now_seconds());
        }

        double now = now_seconds();
        float px, py, pan, tilt;
        if (tracker.aim_point(now, px, py)) {
            calib.pixel_to_angles(px, py, pan, tilt);
            servo->aim(pan, tilt);
            if (tracker.should_fire(now)) {
                servo->fire(tracker.fire_burst_ms());
            }
        } else if (have_last && now - last.t_recv > 1.0) {
            servo->disarm();
            have_last = false;
        }

        // Keep the ESP32 watchdog fed even when we're not aiming.
        if (serial_ptr && now >= next_heartbeat) {
            serial_ptr->heartbeat();
            next_heartbeat = now + 0.2;
        }

        next_tick += tick_dt;
        double sleep = next_tick - now_seconds();
        if (sleep > 0) sleep_seconds(sleep);
        else next_tick = now_seconds();  // fell behind, resync
    }

    servo->disarm();
    std::printf("\n[main] bye\n");
    return 0;
}
