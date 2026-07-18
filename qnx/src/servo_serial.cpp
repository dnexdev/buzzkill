#include "servo_serial.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace buzzkill {

namespace {
speed_t baud_flag(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B115200;
    }
}
} // namespace

SerialServo::SerialServo(const std::string& dev, int baud) {
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "[serial] open %s failed: %s\n",
                     dev.c_str(), std::strerror(errno));
        return;
    }
    termios tio{};
    if (::tcgetattr(fd_, &tio) < 0) {
        std::fprintf(stderr, "[serial] tcgetattr: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1; return;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, baud_flag(baud));
    cfsetospeed(&tio, baud_flag(baud));
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tio) < 0) {
        std::fprintf(stderr, "[serial] tcsetattr: %s\n", std::strerror(errno));
        ::close(fd_); fd_ = -1; return;
    }
    std::printf("[serial] connected to %s @ %d\n", dev.c_str(), baud);
}

SerialServo::~SerialServo() {
    if (fd_ >= 0) {
        write_line("D");
        ::close(fd_);
    }
}

void SerialServo::write_line(const char* s) {
    if (fd_ < 0) return;
    ::write(fd_, s, std::strlen(s));
    ::write(fd_, "\n", 1);
}

void SerialServo::aim(float pan_deg, float tilt_deg) {
    if (std::fabs(pan_deg  - last_pan_)  < 0.25f &&
        std::fabs(tilt_deg - last_tilt_) < 0.25f) return;
    last_pan_ = pan_deg; last_tilt_ = tilt_deg;
    char line[48];
    std::snprintf(line, sizeof(line), "A %.2f %.2f", pan_deg, tilt_deg);
    write_line(line);
    if (!spin_) { write_line("S 1"); spin_ = true; }
}

void SerialServo::fire(int /*duration_ms*/) {
    // ESP32 firmware owns the pusher timing; we just trigger.
    write_line("F");
}

void SerialServo::disarm() {
    write_line("D");
    spin_ = false;
    last_pan_ = last_tilt_ = 1e9f;
}

void SerialServo::heartbeat() { write_line("H"); }

void SerialServo::set_spin(bool on) {
    if (on == spin_) return;
    spin_ = on;
    write_line(on ? "S 1" : "S 0");
}

} // namespace buzzkill
