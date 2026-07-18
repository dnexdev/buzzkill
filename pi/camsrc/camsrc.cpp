// camsrc — capture frames from the QNX Sensor Framework camera and stream
// them as raw BGR bytes to stdout so Python/OpenCV can consume them.
//
// The QNX camera stack (camapi + Sensor Service) is NOT exposed via V4L2 or
// GStreamer libcamerasrc, so cv2.VideoCapture(0) can't work. This binary uses
// the same camapi path as qnx/projects/ai-camera-app and pipes converted BGR
// frames to Python over stdout.
//
// Wire format on stdout:
//   header once at start:  magic="CSRC", u32 width, u32 height, u32 bytesPerPixel
//   then repeatedly:       width * height * bpp bytes of raw BGR
//
// Python spawns this as a subprocess and reads that stream.
//
// Build:  make -C pi/camsrc

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <camera/camera_api.h>
#include <camera/camera_3a.h>

namespace {

std::atomic<bool> g_stop{false};
std::atomic<uint64_t> g_frames{0};

void on_sigint(int) { g_stop.store(true); }

// stderr helpers — stdout is reserved for frame bytes.
#define LOGE(fmt, ...) std::fprintf(stderr, "[camsrc] " fmt "\n", ##__VA_ARGS__)

const char* err_str(camera_error_t e) {
    if (e == CAMERA_EOK)     return "EOK";
    if (e == CAMERA_EPERM)   return "EPERM";
    if (e == CAMERA_EINVAL)  return "EINVAL";
    if (e == CAMERA_EACCESS) return "EACCESS";
    if (e == CAMERA_ENOMEM)  return "ENOMEM";
    if (e == CAMERA_EBUSY)   return "EBUSY";
    if (e == CAMERA_EIO)     return "EIO";
    return "?";
}

// ---- Color conversion -----------------------------------------------------
// The Sensor Framework's ISP for the Pi Camera 3 (IMX708) commonly delivers
// CBYCRY (packed 4:2:2 YUV) or BGR8888 on QNX SDP 8.0. We handle both, plus
// NV12 and RGB8888 as fallbacks.

inline uint8_t clip8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

// BT.601 YUV → BGR.
inline void yuv_to_bgr(int Y, int U, int V, uint8_t& B, uint8_t& G, uint8_t& R) {
    int c = Y - 16;
    int d = U - 128;
    int e = V - 128;
    R = clip8((298 * c + 409 * e + 128) >> 8);
    G = clip8((298 * c - 100 * d - 208 * e + 128) >> 8);
    B = clip8((298 * c + 516 * d + 128) >> 8);
}

// CBYCRY = packed macro-pixel  Cb  Y0  Cr  Y1
// Two pixels per 4 bytes, sharing chroma.
void cbycry_to_bgr(const uint8_t* src, size_t src_stride,
                   int width, int height, uint8_t* bgr_out) {
    for (int j = 0; j < height; ++j) {
        const uint8_t* row = src + j * src_stride;
        uint8_t* out = bgr_out + j * width * 3;
        for (int i = 0; i < width; i += 2) {
            int U  = row[i * 2 + 0];  // Cb
            int Y0 = row[i * 2 + 1];
            int V  = row[i * 2 + 2];  // Cr
            int Y1 = row[i * 2 + 3];
            yuv_to_bgr(Y0, U, V, out[0], out[1], out[2]); out += 3;
            yuv_to_bgr(Y1, U, V, out[0], out[1], out[2]); out += 3;
        }
    }
}

// NV12 = Y plane, then interleaved UV plane. Both are pointers within the
// single framebuf: Y at offset 0, UV at offset `uv_offset`.
void nv12_to_bgr(const uint8_t* framebuf, size_t y_stride,
                 size_t uv_offset,  size_t uv_stride,
                 int width, int height, uint8_t* bgr_out) {
    const uint8_t* y_plane  = framebuf;
    const uint8_t* uv_plane = framebuf + uv_offset;
    for (int j = 0; j < height; ++j) {
        const uint8_t* yr  = y_plane  + j * y_stride;
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

// BGR8888 in memory is B,G,R,A per pixel — drop alpha, keep BGR order.
void bgr8888_to_bgr(const uint8_t* src, size_t src_stride,
                    int width, int height, uint8_t* bgr_out) {
    for (int j = 0; j < height; ++j) {
        const uint8_t* row = src + j * src_stride;
        uint8_t* out = bgr_out + j * width * 3;
        for (int i = 0; i < width; ++i) {
            out[0] = row[0]; out[1] = row[1]; out[2] = row[2];
            out += 3; row += 4;
        }
    }
}

// RGB8888 in memory is R,G,B,A per pixel — need to swap to BGR.
void rgb8888_to_bgr(const uint8_t* src, size_t src_stride,
                    int width, int height, uint8_t* bgr_out) {
    for (int j = 0; j < height; ++j) {
        const uint8_t* row = src + j * src_stride;
        uint8_t* out = bgr_out + j * width * 3;
        for (int i = 0; i < width; ++i) {
            out[0] = row[2]; out[1] = row[1]; out[2] = row[0];
            out += 3; row += 4;
        }
    }
}

struct StreamState {
    int width  = 0;
    int height = 0;
    bool header_sent = false;
    std::vector<uint8_t> bgr;
};

StreamState g_state;

void send_header(int w, int h) {
    // 4-byte magic + 3 x u32 little-endian native.
    // Python reads with struct.unpack("<4sIII", ...).
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

// Viewfinder callback. Runs on a camapi thread. Convert → stdout.
void viewfinder_cb(camera_handle_t /*handle*/, camera_buffer_t* buf, void* /*arg*/) {
    if (!buf || g_stop.load() || !buf->framebuf) return;

    int w = 0, h = 0;
    switch (buf->frametype) {
        case CAMERA_FRAMETYPE_NV12:
        case CAMERA_FRAMETYPE_NV16:
        case CAMERA_FRAMETYPE_NV24_Y12:
            w = static_cast<int>(buf->framedesc.nv12.width);
            h = static_cast<int>(buf->framedesc.nv12.height);
            break;
        case CAMERA_FRAMETYPE_CBYCRY:
            w = static_cast<int>(buf->framedesc.cbycry.width);
            h = static_cast<int>(buf->framedesc.cbycry.height);
            break;
        case CAMERA_FRAMETYPE_RGB8888:
            w = static_cast<int>(buf->framedesc.rgb8888.width);
            h = static_cast<int>(buf->framedesc.rgb8888.height);
            break;
        case CAMERA_FRAMETYPE_BGR8888:
            w = static_cast<int>(buf->framedesc.bgr8888.width);
            h = static_cast<int>(buf->framedesc.bgr8888.height);
            break;
        default:
            w = g_state.width;
            h = g_state.height;
            break;
    }
    if (w <= 0 || h <= 0) return;

    if (!g_state.header_sent) {
        g_state.width  = w;
        g_state.height = h;
        g_state.bgr.assign(static_cast<size_t>(w) * h * 3, 0);
        send_header(w, h);
        g_state.header_sent = true;
        LOGE("streaming %dx%d, frametype=%d", w, h, (int)buf->frametype);
    }

    switch (buf->frametype) {
        case CAMERA_FRAMETYPE_CBYCRY: {
            const auto& d = buf->framedesc.cbycry;
            cbycry_to_bgr(buf->framebuf, d.stride, w, h, g_state.bgr.data());
            break;
        }
        case CAMERA_FRAMETYPE_NV12:
        case CAMERA_FRAMETYPE_NV16:
        case CAMERA_FRAMETYPE_NV24_Y12: {
            const auto& d = buf->framedesc.nv12;
            nv12_to_bgr(buf->framebuf, d.stride, d.uv_offset, d.uv_stride,
                        w, h, g_state.bgr.data());
            break;
        }
        case CAMERA_FRAMETYPE_BGR8888: {
            const auto& d = buf->framedesc.bgr8888;
            bgr8888_to_bgr(buf->framebuf, d.stride, w, h, g_state.bgr.data());
            break;
        }
        case CAMERA_FRAMETYPE_RGB8888: {
            const auto& d = buf->framedesc.rgb8888;
            rgb8888_to_bgr(buf->framebuf, d.stride, w, h, g_state.bgr.data());
            break;
        }
        default:
            LOGE("unsupported frametype %d, cannot convert to BGR",
                 (int)buf->frametype);
            return;
    }

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

// Try each format until one is accepted. Returns the first success.
camera_error_t try_set_format(camera_handle_t handle, int w, int h) {
    camera_error_t err;
    const struct { camera_frametype_t fmt; const char* name; } tries[] = {
        { CAMERA_FRAMETYPE_CBYCRY,   "CBYCRY"   },
        { CAMERA_FRAMETYPE_BGR8888,  "BGR8888"  },
        { CAMERA_FRAMETYPE_NV12,     "NV12"     },
        { CAMERA_FRAMETYPE_RGB8888,  "RGB8888"  },
    };
    for (const auto& t : tries) {
        err = camera_set_vf_property(
            handle,
            CAMERA_IMGPROP_WIDTH,  w,
            CAMERA_IMGPROP_HEIGHT, h,
            CAMERA_IMGPROP_FORMAT, t.fmt);
        if (err == CAMERA_EOK) {
            LOGE("using %s @ %dx%d", t.name, w, h);
            return CAMERA_EOK;
        }
        LOGE("format %s not accepted: %s", t.name, err_str(err));
    }
    return err;
}

} // namespace

int main(int argc, char** argv) {
    int unit = 1;
    int req_w = 640;
    int req_h = 480;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--unit" && i + 1 < argc) unit  = std::atoi(argv[++i]);
        else if (a == "--w"    && i + 1 < argc) req_w = std::atoi(argv[++i]);
        else if (a == "--h"    && i + 1 < argc) req_h = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help") {
            std::fprintf(stderr,
              "usage: camsrc [--unit N] [--w N] [--h N]\n"
              "  emits BGR frames to stdout, log to stderr\n");
            return 0;
        }
    }

    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);
    std::signal(SIGPIPE, on_sigint);

    LOGE("opening camera unit %d", unit);
    camera_handle_t handle;
    camera_error_t err = camera_open(static_cast<camera_unit_t>(unit),
                                     CAMERA_MODE_RW, &handle);
    if (err != CAMERA_EOK) {
        LOGE("camera_open(%d): %s", unit, err_str(err));
        return 1;
    }

    err = try_set_format(handle, req_w, req_h);
    if (err != CAMERA_EOK) {
        LOGE("no supported format worked");
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
