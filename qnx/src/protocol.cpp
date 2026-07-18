#include "protocol.h"

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <string>

namespace buzzkill {

namespace {

const char* find_key(const char* s, const char* end, const char* key) {
    std::string needle = std::string("\"") + key + "\"";
    const char* p = s;
    while (p < end) {
        const char* hit = std::strstr(p, needle.c_str());
        if (!hit || hit >= end) return nullptr;
        const char* c = hit + needle.size();
        while (c < end && std::isspace(static_cast<unsigned char>(*c))) ++c;
        if (c < end && *c == ':') return c + 1;
        p = hit + 1;
    }
    return nullptr;
}

bool read_number(const char* s, const char* end, double& out) {
    while (s < end && std::isspace(static_cast<unsigned char>(*s))) ++s;
    if (s >= end) return false;
    char* endp = nullptr;
    out = std::strtod(s, &endp);
    return endp != s;
}

bool read_bool(const char* s, const char* end, bool& out) {
    while (s < end && std::isspace(static_cast<unsigned char>(*s))) ++s;
    if (end - s >= 4 && std::strncmp(s, "true", 4) == 0) { out = true; return true; }
    if (end - s >= 5 && std::strncmp(s, "false", 5) == 0) { out = false; return true; }
    return false;
}

} // namespace

bool parse_target(const char* buf, size_t len, Target& out) {
    const char* end = buf + len;
    double d;

    const char* p;
    if (!(p = find_key(buf, end, "t"))  || !read_number(p, end, d)) return false;
    out.t_send = d;
    if (!(p = find_key(buf, end, "fw")) || !read_number(p, end, d)) return false;
    out.frame_w = static_cast<uint16_t>(d);
    if (!(p = find_key(buf, end, "fh")) || !read_number(p, end, d)) return false;
    out.frame_h = static_cast<uint16_t>(d);
    if (!(p = find_key(buf, end, "x"))  || !read_number(p, end, d)) return false;
    out.x = static_cast<float>(d);
    if (!(p = find_key(buf, end, "y"))  || !read_number(p, end, d)) return false;
    out.y = static_cast<float>(d);
    if (!(p = find_key(buf, end, "vx")) || !read_number(p, end, d)) return false;
    out.vx = static_cast<float>(d);
    if (!(p = find_key(buf, end, "vy")) || !read_number(p, end, d)) return false;
    out.vy = static_cast<float>(d);
    if (!(p = find_key(buf, end, "conf")) || !read_number(p, end, d)) return false;
    out.confidence = static_cast<float>(d);
    bool b;
    if (!(p = find_key(buf, end, "det")) || !read_bool(p, end, b)) return false;
    out.detected = b;
    return true;
}

} // namespace buzzkill
