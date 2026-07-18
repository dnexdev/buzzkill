#pragma once

#include <string>

namespace buzzkill {

// 4-corner bilinear map from pixel (x,y) to (pan_deg, tilt_deg).
class Calibration {
public:
    // Loads JSON produced by laptop/calibrate.py. Returns false on error.
    bool load(const std::string& path);

    // If not loaded, uses a default linear map spanning ± angle_span.
    void set_default(int frame_w, int frame_h, float angle_span_deg = 30.f);

    // Convert pixel to servo angles.
    void pixel_to_angles(float px, float py, float& pan, float& tilt) const;

    int frame_w() const { return fw_; }
    int frame_h() const { return fh_; }

private:
    // 4 corners in order TL, TR, BR, BL
    struct Pt { float px, py, pan, tilt; };
    Pt corners_[4]{};
    int fw_ = 640, fh_ = 480;
    bool loaded_ = false;
};

} // namespace buzzkill
