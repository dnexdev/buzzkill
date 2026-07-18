#include "receiver.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace buzzkill {

namespace {
double now_seconds() {
    timeval tv;
    gettimeofday(&tv, nullptr);
    return tv.tv_sec + tv.tv_usec * 1e-6;
}
} // namespace

Receiver::~Receiver() {
    if (fd_ >= 0) ::close(fd_);
}

bool Receiver::bind(int port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::fprintf(stderr, "socket: %s\n", std::strerror(errno));
        return false;
    }
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "bind %d: %s\n", port, std::strerror(errno));
        return false;
    }
    return true;
}

bool Receiver::poll(Target& out) {
    if (fd_ < 0) return false;
    char buf[1024];
    // Drain the socket, keep the newest packet only.
    bool got = false;
    Target latest{};
    for (;;) {
        ssize_t n = ::recvfrom(fd_, buf, sizeof(buf) - 1, 0, nullptr, nullptr);
        if (n <= 0) break;
        buf[n] = '\0';
        Target t{};
        if (parse_target(buf, static_cast<size_t>(n), t)) {
            t.t_recv = now_seconds();
            latest = t;
            got = true;
        }
    }
    if (got) out = latest;
    return got;
}

} // namespace buzzkill
