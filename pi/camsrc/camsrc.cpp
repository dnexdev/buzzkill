// camsrc — capture frames from the QNX Sensor Framework camera and stream
// them as raw BGR bytes to stdout, so Python/OpenCV can consume them.
//
// This exists because the QNX camera stack (camapi + Sensor Service) is NOT
// exposed via V4L2 or GStreamer libcamerasrc, so cv2.VideoCapture(0) fails.
// We take the plain camapi path (same as qnx/projects/ai-camera-app) and
// pipe frames to Python over stdout.
//
// Wire format on stdout:
//   header once at start:  magic="CSRC", u32 width, u32 height, u32 bytesPerPixel
//   then repeatedly:       width * height * bpp bytes of raw BGR
//
// Python side spawns this as a subprocess and reads that stream.
//
// Build:  make -C pi/camsrc
// Run:    ./camsrc | your_consumer
// Args:   --unit N   camera unit (default 1)
//         --w N      requested viewfinder width (default 640)
//         --h N      requested viewfinder height (default 480)

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

// QNX Sensor Framework camera API. Headers live at /usr/include/camera/*
#include <camera/camera_api.h>
#include <camera/camera_3a.h>

namespace {

std::atomic<bool> g_stop{false};
std::atomic<uint64_t> g_frames{0};

void on_sigint(int) { g_stop.store(true); }

// stderr helpers — stdout is reserved for frame data.
#define LOGE(fmt, ...) std::fprintf(stderr, "[camsrc] " fmt "\n", ##__VA_ARGS__)

const char* err_str(camera_error_t e) {
    switch (e) {
        case CAMERA_EOK:        return "EOK";
        case CAMERA_EAGAIN:     return "EAGAIN";
        case CAMERA_EINVAL:     return "EINVAL";
        case CAMERA_ENODEV:     return "ENODEV";
        case CAMERA_EMFILE:     return "EMFILE";
        case CAMERA_EBADF:      return "EBADF";
        case CAMERA_EACCES:     return "EACCES";
        case CAMERA_EBADR:      return "EBADR";
        case CAMERA_ENOENT:     return "ENOENT";
        case CAMERA_ENOMEM:     return "ENOMEM";
        case CAMERA_EOPNOTSUPP: return "EOPNOTSUPP";
        case CAMERA_ETIMEDOUT:  return "ETIMEDOUT";
        default:                return "?";
    }
}

// ---- Frame conversion ------------------------------------------------------
//
// The Sensor Framework can deliver frames in several formats depending on the
// sensor and the ISP. IMX708 (Pi Camera Module 3) typically produces YUV
// (NV12 or YUY2) after ISP. We convert to packed BGR for Python.

// Clamp a signed int into 0..255.
inline uint8_t clip8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

// YUV -> BGR per BT.601. Standard integer approximation.
inline void yuv_to_bgr(int Y, int U, int V, uint8_t& B, uint8_t& G, uint8_t& R) {
    int c = Y - 16;
    int d = U - 128;
    int e = V - 128;
    R = clip8((298 * c + 409 * e + 128) >> 8);
    G = clip8((298 * c - 100 * d - 208 * e + 128) >> 8);
    B = clip8((298 * c + 516 * d + 128) >> 8);
}

// Convert NV12 (Y plane + interleaved UV plane) to packed BGR.
// Both planes must be provided with their strides.
void nv12_to_bgr(const uint8_t* y_plane, size_t y_stride,
                 const uint8_t* uv_plane, size_t uv_stride,
                 int width, int height, uint8_t* bgr_out) {
    for (int j = 0; j < height; ++j) {
        const uint8_t* yr = y_plane + j * y_stride;
        const uint8_t* uvr = uv_plane + (j / 2) * uv_stride;
        uint8_t* out = bgr_out + j * width * 3;
        for (int i = 0; i < width; ++i) {
            int Y = yr[i];
            int U = uvr[(i / 2) * 2 + 0];
            int V = uvr[(i / 2) * 2 + 1];
            yuv_to_bgr(Y, U, V, out[0], out[1], out[2]);
            out += 3;
        }
    }
}

// Convert YUY2 (packed YUYV, 2 bytes per pixel) to packed BGR.
void yuy2_to_bgr(const uint8_t* src, size_t src_stride,
                 int width, int height, uint8_t* bgr_out) {
    for (int j = 0; j < height; ++j) {
        const uint8_t* row = src + j * src_stride;
        uint8_t* out = bgr_out + j * width * 3;
        for (int i = 0; i < width; i += 2) {
            int Y0 = row[i * 2 + 0];
            int U  = row[i * 2 + 1];
            int Y1 = row[i * 2 + 2];
            int V  = row[i * 2 + 3];
            yuv_to_bgr(Y0, U, V, out[0], out[1], out[2]); out += 3;
            yuv_to_bgr(Y1, U, V, out[0], out[1], out[2]); out += 3;
        }
    }
}

// State shared with the viewfinder callback. Only the callback writes here.
struct StreamState {
    int width  = 0;
    int height = 0;
    bool header_sent = false;
    std::vector<uint8_t> bgr;
};

StreamState g_state;

void send_header(int w, int h) {
    // 4-byte magic + 3 x u32 in little-endian native order.
    // Python side reads with struct.unpack("<4sIII", ...).
    const char magic[4] = {'C','S','R','C'};
    uint32_t W = static_cast<uint32_t>(w);
    uint32_t H = static_cast<uint32_t>(h);
    uint32_t BPP = 3;
    std::fwrite(magic, 1, 4, stdout);
    std::fwrite(&W,   sizeof(W), 1, stdout);
    std::fwrite(&H,   sizeof(H), 1, stdout);
    std::fwrite(&BPP, sizeof(BPP), 1, stdout);
    std::fflush(stdout);
}

// Called by camapi for every viewfinder frame. Convert to BGR, write to stdout.
void viewfinder_cb(camera_handle_t /*handle*/, camera_buffer_t* buf, void* /*arg*/) {
    if (!buf || g_stop.load()) return;

    int w = 0, h = 0;
    // Union access varies by frametype; pull dims from whichever field matches.
    switch (buf->frametype) {
        case CAMERA_FRAMETYPE_NV12:
            w = buf->framedesc.nv12.width;
            h = buf->framedesc.nv12.height;
            break;
        case CAMERA_FRAMETYPE_YCBCR422P:
        case CAMERA_FRAMETYPE_CBYCRY:
            w = buf->framedesc.cbycry.width;
            h = buf->framedesc.cbycry.height;
            break;
        case CAMERA_FRAMETYPE_RGB8888:
            w = buf->framedesc.rgb8888.width;
            h = buf->framedesc.rgb8888.height;
            break;
        default:
            // Fall back to best-effort dims — some format we didn't code up.
            w = g_state.width;
            h = g_state.height;
            break;
    }
    if (w <= 0 || h <= 0) return;

    if (!g_state.header_sent) {
        g_state.width = w;
        g_state.height = h;
        g_state.bgr.assign(static_cast<size_t>(w) * h * 3, 0);
        send_header(w, h);
        g_state.header_sent = true;
        LOGE("streaming %dx%d, frametype=%d", w, h, (int)buf->frametype);
    }

    // Convert to BGR based on frame type.
    switch (buf->frametype) {
        case CAMERA_FRAMETYPE_NV12: {
            const auto& d = buf->framedesc.nv12;
            nv12_to_bgr(d.y, d.stride, d.uv, d.uv_stride,
                        w, h, g_state.bgr.data());
            break;
        }
        case CAMERA_FRAMETYPE_CBYCRY: {
            const auto& d = buf->framedesc.cbycry;
            yuy2_to_bgr(d.buffer, d.stride, w, h, g_state.bgr.data());
            break;
        }
        case CAMERA_FRAMETYPE_RGB8888: {
            const auto& d = buf->framedesc.rgb8888;
            // BGR8888 is 4 bytes/pixel BGRA — drop alpha.
            for (int j = 0; j < h; ++j) {
                const uint8_t* row = d.buffer + j * d.stride;
                uint8_t* out = g_state.bgr.data() + j * w * 3;
                for (int i = 0; i < w; ++i) {
                    out[0] = row[0]; out[1] = row[1]; out[2] = row[2];
                    out += 3; row += 4;
                }
            }
            break;
        }
        default:
            LOGE("unsupported frametype %d — cannot convert to BGR",
                 (int)buf->frametype);
            return;
    }

    // Write one frame. stdout is line-buffered by default; we force full flush.
    const size_t nbytes = static_cast<size_t>(w) * h * 3;
    if (std::fwrite(g_state.bgr.data(), 1, nbytes, stdout) != nbytes) {
        LOGE("stdout write short — consumer went away, stopping");
        g_stop.store(true);
    }
    std::fflush(stdout);
    g_frames.fetch_add(1);
}

void status_cb(camera_handle_t /*h*/, camera_devstatus_t status,
               uint16_t extra, void* /*arg*/) {
    LOGE("status: status=%d extra=%u", (int)status, (unsigned)extra);
}

} // namespace

int main(int argc, char** argv) {
    int unit = 1;
    int req_w = 640;
    int req_h = 480;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--unit" && i + 1 < argc) unit = std::atoi(argv[++i]);
        else if (a == "--w" && i + 1 < argc) req_w = std::atoi(argv[++i]);
        else if (a == "--h" && i + 1 < argc) req_h = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::fprintf(stderr,
              "usage: camsrc [--unit N] [--w N] [--h N]\n"
              "  emits BGR frames to stdout, log to stderr\n");
            return 0;
        }
    }

    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);
    std::signal(SIGPIPE, on_sigint);   // consumer went away

    LOGE("opening camera unit %d", unit);
    camera_handle_t handle;
    camera_error_t err = camera_open(static_cast<camera_unit_t>(unit),
                                     CAMERA_MODE_RW, &handle);
    if (err != CAMERA_EOK) {
        LOGE("camera_open(%d) failed: %s", unit, err_str(err));
        return 1;
    }

    // Request viewfinder resolution. If the sensor can't hit exactly this, the
    // callback will report the real dims via the header we send Python.
    err = camera_set_vf_property(
        handle,
        CAMERA_IMGPROP_WIDTH,  req_w,
        CAMERA_IMGPROP_HEIGHT, req_h,
        CAMERA_IMGPROP_FORMAT, CAMERA_FRAMETYPE_NV12);
    if (err != CAMERA_EOK) {
        LOGE("set NV12 %dx%d failed (%s), trying CBYCRY", req_w, req_h, err_str(err));
        err = camera_set_vf_property(
            handle,
            CAMERA_IMGPROP_WIDTH,  req_w,
            CAMERA_IMGPROP_HEIGHT, req_h,
            CAMERA_IMGPROP_FORMAT, CAMERA_FRAMETYPE_CBYCRY);
    }
    if (err != CAMERA_EOK) {
        LOGE("set CBYCRY failed (%s), trying RGB8888", err_str(err));
        err = camera_set_vf_property(
            handle,
            CAMERA_IMGPROP_WIDTH,  req_w,
            CAMERA_IMGPROP_HEIGHT, req_h,
            CAMERA_IMGPROP_FORMAT, CAMERA_FRAMETYPE_RGB8888);
    }
    if (err != CAMERA_EOK) {
        LOGE("no supported format worked: %s", err_str(err));
        camera_close(handle);
        return 2;
    }

    err = camera_start_viewfinder(handle, viewfinder_cb, status_cb, nullptr);
    if (err != CAMERA_EOK) {
        LOGE("start_viewfinder: %s", err_str(err));
        camera_close(handle);
        return 3;
    }
    LOGE("viewfinder started");

    auto next_report = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    uint64_t last_count = 0;
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto now = std::chrono::steady_clock::now();
        if (now >= next_report) {
            uint64_t c = g_frames.load();
            LOGE("fps~%llu (total %llu)",
                 static_cast<unsigned long long>(c - last_count),
                 static_cast<unsigned long long>(c));
            last_count = c;
            next_report = now + std::chrono::seconds(1);
        }
    }

    LOGE("stopping");
    camera_stop_viewfinder(handle);
    camera_close(handle);
    return 0;
}
