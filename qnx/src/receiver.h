#pragma once

#include "protocol.h"

namespace buzzkill {

class Receiver {
public:
    Receiver() = default;
    ~Receiver();
    bool bind(int port);
    // Non-blocking: returns true if a fresh packet was parsed into `out`.
    bool poll(Target& out);
private:
    int fd_ = -1;
};

} // namespace buzzkill
