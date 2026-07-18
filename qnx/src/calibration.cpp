#include "calibration.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace buzzkill {

namespace {

// Very small JSON scanner tuned to our known shape. Not a general parser.
double find_num(const std::string& s, size_t& pos, const char* key) {
    std::string needle = std::string("\"") + key + "\"";
    size_t k = s.find(needle, pos);
    if (k == std::string::npos) return 0;
    size_t c = s.find(':', k);
    if (c == std::string::npos) return 0;
    pos = c + 1;
    return std::strtod(s.c_str() + pos, nullptr);
}

const char* corner_name(int i) {
    switch (i) {
        case 0: return "top-left";
        case 1: return "top-right";
        case 2: return "bottom-right";
        case 3: return "bottom-left";
    }
    return "?";
}

} // namespace

bool Calibration::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "calibration: cannot open %s\n", path.c_str());
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    std::string s = ss.str();

    size_t pos = 0;
    fw_ = static_cast<int>(find_num(s, pos, "frame_w"));
    fh_ = static_cast<int>(find_num(s, pos, "frame_h"));

    for (int i = 0; i < 4; ++i) {
        size_t k = s.find(corner_name(i), pos);
        if (k == std::string::npos) {
            std::fprintf(stderr, "calibration: missing %s\n", corner_name(i));
            return false;
        }
        pos = k;
        corners_[i].px   = static_cast<float>(find_num(s, pos, "px"));
        corners_[i].py   = static_cast<float>(find_num(s, pos, "py"));
        corners_[i].pan  = static_cast<float>(find_num(s, pos, "pan"));
        corners_[i].tilt = static_cast<float>(find_num(s, pos, "tilt"));
    }
    loaded_ = true;
    std::printf("[calib] loaded %s (%dx%d)\n", path.c_str(), fw_, fh_);
    return true;
}

void Calibration::set_default(int frame_w, int frame_h, float span) {
    fw_ = frame_w; fh_ = frame_h;
    corners_[0] = {0.f,                (float)frame_h * 0.f, -span,  span};
    corners_[1] = {(float)frame_w,     (float)frame_h * 0.f,  span,  span};
    corners_[2] = {(float)frame_w,     (float)frame_h,        span, -span};
    corners_[3] = {0.f,                (float)frame_h,       -span, -span};
    loaded_ = true;
}

void Calibration::pixel_to_angles(float px, float py, float& pan, float& tilt) const {
    // Normalize u,v ∈ [0,1] and bilinearly blend the four corners' angles.
    float u = (fw_ > 0) ? std::clamp(px / (float)fw_, 0.f, 1.f) : 0.f;
    float v = (fh_ > 0) ? std::clamp(py / (float)fh_, 0.f, 1.f) : 0.f;
    // corners: TL=0 TR=1 BR=2 BL=3
    auto blend = [&](float a, float b, float c, float d) {
        float top = a * (1 - u) + b * u;
        float bot = d * (1 - u) + c * u;
        return top * (1 - v) + bot * v;
    };
    pan  = blend(corners_[0].pan,  corners_[1].pan,
                 corners_[2].pan,  corners_[3].pan);
    tilt = blend(corners_[0].tilt, corners_[1].tilt,
                 corners_[2].tilt, corners_[3].tilt);
}

} // namespace buzzkill
