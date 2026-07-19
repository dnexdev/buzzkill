"""Motion + darkness detection pipeline. Drives the turret directly.

Optimized for the demo: black paper mosquitoes swinging on strings against
a lighter background. The pipeline is:

  frame -> MOG2 motion mask -> AND -> darkness mask -> morph -> contours -> centroid

The AND is the key trick: motion alone triggers on hands and walking people;
darkness alone triggers on any dark object (a chair leg). Together they only
fire when something dark is moving.

Every processed frame that finds a target also runs a P-only controller per
axis on the pixel error between the target and the frame center, capped at
--max-step degrees, and sends the result straight out over serial.

Usage:
  python3 detect.py --source 0
  python3 detect.py --source 0 --dry-run          # print aim commands, no serial
  python3 detect.py --source 0 --serial /dev/ser1
  python3 detect.py --source 0 --show-mask        # tune the darkness threshold
"""
from __future__ import annotations

import argparse
import http.server
import signal
import socketserver
import threading
import time

import cv2
import numpy as np


class MjpegServer:
    """Serve the latest annotated frame as MJPEG so a browser on the laptop
    can view what the (headless) Pi detector is seeing. Two endpoints:
      /         landing page auto-loading /stream.mjpg
      /stream.mjpg  multipart JPEG stream
    """

    def __init__(self, port: int):
        self._port = port
        self._latest_jpeg = None
        self._cv = threading.Condition()
        server = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass  # silence access log

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = (
                        b"<html><body style='margin:0;background:#111'>"
                        b"<img src='/stream.mjpg' style='width:100%'/>"
                        b"</body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith("/stream"):
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while True:
                            with server._cv:
                                server._cv.wait(timeout=1.0)
                                jpg = server._latest_jpeg
                            if jpg is None:
                                continue
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        return
                    return
                self.send_error(404)

        class ThreadedServer(socketserver.ThreadingMixIn,
                             http.server.HTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        self._httpd = ThreadedServer(("0.0.0.0", port), Handler)
        t = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        t.start()

    def publish(self, frame_bgr):
        ok, buf = cv2.imencode(".jpg", frame_bgr,
                               [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        with self._cv:
            self._latest_jpeg = buf.tobytes()
            self._cv.notify_all()

    @property
    def port(self):
        return self._port

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SerialSink:
    def __init__(self, dev: str, baud: int, wait_boot: float):
        try:
            import serial  # pyserial
            self._port = serial.Serial(dev, baud, timeout=0, write_timeout=0.5)
            self._use_pyserial = True
        except ImportError:
            # QNX has no pyserial. Set the baud rate with stty, then write
            # straight to the device file.
            print(f"[detect] pyserial not found — writing directly to {dev}",
                  flush=True)
            import subprocess
            try:
                with open(dev, "rb") as devfd:
                    subprocess.run(["stty", f"baud={baud}"], stdin=devfd,
                                   check=False)
            except Exception as e:
                print(f"[detect] stty baud set failed ({e}); "
                      f"continuing with device defaults", flush=True)
            self._port = open(dev, "wb", buffering=0)
            self._use_pyserial = False

        time.sleep(wait_boot)  # let the board finish any reset-on-open
        if self._use_pyserial:
            try:
                self._port.reset_input_buffer()
            except Exception:
                pass

    def send(self, line: str) -> None:
        self._port.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        try:
            self._port.close()
        except Exception:
            pass


class PrintSink:
    def send(self, line: str) -> None:
        print(f"[aim] > {line}", flush=True)

    def close(self) -> None:
        pass


class CamsrcCapture:
    """Reads BGR frames from the camsrc subprocess (QNX camapi bridge).

    camsrc writes: 16-byte header ("CSRC" + u32 w + u32 h + u32 bpp)
    then repeatedly: w*h*bpp bytes of raw BGR.
    """
    def __init__(self, cmd):
        import subprocess, struct
        self._struct = struct
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
        )
        header = self._read_exact(16)
        if header is None:
            err = self._proc.stderr.read().decode("utf-8", errors="replace")
            raise SystemExit(f"camsrc died before header:\n{err}")
        magic = header[:4]
        if magic != b"CSRC":
            raise SystemExit(f"camsrc bad magic: {magic!r}")
        self.width, self.height, self.bpp = struct.unpack("<III", header[4:16])
        self._frame_size = self.width * self.height * self.bpp
        if self.bpp != 3:
            raise SystemExit(f"camsrc bpp {self.bpp} != 3, cannot use")
        # Drain stderr in a background thread so the pipe doesn't fill and stall.
        import threading
        def _drain():
            for line in self._proc.stderr:
                pass
        threading.Thread(target=_drain, daemon=True).start()

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self._proc.stdout.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def isOpened(self):
        return self._proc.poll() is None

    def read(self):
        raw = self._read_exact(self._frame_size)
        if raw is None:
            return False, None
        # .copy() makes the array writable; frombuffer views are read-only
        # and OpenCV can't draw on read-only Mats.
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(
            (self.height, self.width, 3)).copy()
        return True, arr

    def set(self, *_):
        # No-op: dimensions are fixed by camsrc.
        return False

    def release(self):
        try:
            self._proc.terminate()
            self._proc.wait(timeout=1.0)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass


def open_camera(source, width, height):
    # Special source: launch our QNX camapi bridge and read raw BGR from its stdout.
    if str(source) == "camsrc":
        import os
        exe = os.path.join(os.path.dirname(__file__), "camsrc", "camsrc")
        if not os.path.exists(exe):
            raise SystemExit(
                f"camsrc binary not found at {exe}. "
                "build with:  make -C pi/camsrc")
        cap = CamsrcCapture([exe, "--w", str(width), "--h", str(height)])
        return cap

    is_pipeline = isinstance(source, str) and (" " in source or "!" in source)
    src = int(source) if (not is_pipeline and str(source).isdigit()) else source
    if is_pipeline:
        cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {source}")
    if not is_pipeline:
        # These properties only apply to V4L2/webcam. GStreamer pipelines
        # already encode dimensions in the source string.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # Warmup — first frames from CSI cameras are often garbage.
    for _ in range(5):
        cap.read()
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0",
                    help="0 for CSI/webcam, or URL")
    ap.add_argument("--width",  type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--zoom", type=float, default=1.0,
                    help="center-crop factor (digital zoom). 2.0 keeps the "
                         "middle half of the frame in each dimension. 1.0 = off.")

    # Auto-aim: P-only controller per axis, straight out over serial.
    ap.add_argument("--serial", default="/dev/ser1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dry-run", action="store_true",
                    help="print aim commands instead of writing to serial")
    ap.add_argument("--kp-pan", type=float, default=0.02, help="pan gain, deg/px")
    ap.add_argument("--kp-tilt", type=float, default=0.02, help="tilt gain, deg/px")
    ap.add_argument("--max-step", type=float, default=5.0,
                    help="max degrees per control tick (speed cap)")
    ap.add_argument("--move-cooldown", type=float, default=0.25,
                    help="min seconds between move commands. keeps the turret "
                         "from machine-gunning steps at frame rate.")
    ap.add_argument("--wait-boot", type=float, default=2.0)
    ap.add_argument("--min-confidence", type=float, default=0.15,
                    help="minimum target confidence to acquire aim. confidence "
                         "is a product of four 0-1 scores (dark, white ring, "
                         "contrast, fill), so real targets score ~0.2-0.45; "
                         "0.5 rejects nearly everything. 0 disables the gate.")

    # Tuned defaults for the demo (black mosquitoes on white background).
    # Loose enough to catch small (far away) and blurred (moving) mosquitoes.
    ap.add_argument("--dark-threshold", type=int, default=110)
    ap.add_argument("--adaptive", action="store_true",
                    help="use adaptive thresholding instead of global --dark-threshold. "
                         "handles lighting variation automatically — brightly lit rooms, "
                         "dim rooms, uneven lighting all look the same to the pipeline.")
    ap.add_argument("--adaptive-block", type=int, default=51,
                    help="[adaptive] neighborhood size in pixels; must be odd. "
                         "larger = smoother, smaller = more local variation captured.")
    ap.add_argument("--adaptive-c", type=int, default=10,
                    help="[adaptive] constant subtracted from local mean. "
                         "higher = fewer dark pixels detected.")
    ap.add_argument("--min-area", type=int, default=20)
    ap.add_argument("--max-area", type=int, default=3000)
    ap.add_argument("--max-compactness", type=float, default=1.0,
                    help="4πA/P². disabled by default (1.0). tighten to reject smooth blobs.")
    ap.add_argument("--history", type=int, default=200)
    ap.add_argument("--var-threshold", type=int, default=32)
    ap.add_argument("--shadow-threshold", type=float, default=0.5)
    ap.add_argument("--motion-dilate", type=int, default=15)
    ap.add_argument("--close-kernel", type=int, default=11)
    ap.add_argument("--min-fill", type=float, default=0.35)

    # Demo mode: guaranteed white background.
    ap.add_argument("--white-bg", action="store_true",
                    help="skip MOG2 and use pure darkness threshold.")
    ap.add_argument("--min-motion", type=float, default=3.0)
    ap.add_argument("--min-bg-brightness", type=int, default=180)
    ap.add_argument("--bg-ring-px", type=int, default=6,
                    help="ring width for background check. small enough not to "
                         "reach into adjacent mosquitoes on a dense sheet.")
    ap.add_argument("--min-contrast", type=int, default=100)
    ap.add_argument("--white-cutoff", type=int, default=200,
                    help="pixel > this counts as white for ring-purity check")
    ap.add_argument("--min-white-fraction", type=float, default=0.35,
                    help="ring around blob must be this fraction white or higher. "
                         "large non-white regions (face, hair) tank this below 0.3.")

    # Shape filters — loose; a moving mosquito is a smeared streak, not a silhouette.
    ap.add_argument("--aspect-min", type=float, default=0.0)
    ap.add_argument("--aspect-max", type=float, default=99.0)
    ap.add_argument("--min-solidity", type=float, default=0.10)
    ap.add_argument("--max-solidity", type=float, default=0.99)
    ap.add_argument("--min-extent", type=float, default=0.05)
    ap.add_argument("--max-extent", type=float, default=0.95)
    ap.add_argument("--template", default="template.npy",
                    help="path to saved shape template (auto-loaded if present)")
    ap.add_argument("--shape-tolerance", type=float, default=999.0)
    ap.add_argument("--shape-min-area", type=int, default=200,
                    help="only apply shape-template match on blobs bigger than this. "
                         "small blobs have too few points for stable Hu moments.")

    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--show-mask", action="store_true",
                    help="show combined detection mask alongside frame")
    ap.add_argument("--debug", action="store_true",
                    help="print detection stats and rejection reasons every 2s. "
                         "essential when running headless (no cv2.imshow).")
    ap.add_argument("--save-debug", type=int, default=0, metavar="N",
                    help="every N frames, save the current annotated frame and mask "
                         "to /tmp/buzzkill_frame.png and /tmp/buzzkill_mask.png. "
                         "scp these off the Pi to see what the detector sees. "
                         "0 = disabled.")
    ap.add_argument("--stream-port", type=int, default=0, metavar="PORT",
                    help="serve annotated frames as MJPEG over HTTP on this port. "
                         "open http://<pi-ip>:PORT in a browser on your laptop. "
                         "0 = disabled.")
    args = ap.parse_args()

    stop = [False]
    def on_sigint(*_):
        stop[0] = True
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    cap = open_camera(args.source, args.width, args.height)
    sink = PrintSink() if args.dry_run else SerialSink(args.serial, args.baud, args.wait_boot)

    mjpeg = None
    if args.stream_port > 0:
        mjpeg = MjpegServer(args.stream_port)
        print(f"[detect] MJPEG stream on http://<pi-ip>:{args.stream_port}/")

    # Optional shape template — loaded at boot, replaced at runtime with T key.
    template_contour = None
    if args.template:
        try:
            template_contour = np.load(args.template, allow_pickle=False)
            print(f"[detect] loaded template from {args.template} "
                  f"({len(template_contour)} points)")
        except FileNotFoundError:
            print(f"[detect] no template at {args.template}. "
                  f"press T while target locked to capture one.")
        except Exception as e:
            print(f"[detect] template load failed: {e}")

    bg = cv2.createBackgroundSubtractorMOG2(
        history=args.history,
        varThreshold=args.var_threshold,
        detectShadows=True,   # marks shadow pixels as 127 so we can drop them
    )
    # Shadows drift lower/higher based on lighting — 0.5 rejects most soft shadows,
    # 0.7 is stricter (rejects harsher shadows too, at slight cost to real detections).
    bg.setShadowThreshold(args.shadow_threshold)
    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                             (args.close_kernel, args.close_kernel))
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                              (args.motion_dilate, args.motion_dilate))

    prev_pt = None
    prev_ts = None
    lost_frames = 0
    prev_centroids = []  # white-bg mode: candidate centroids from last frame
    had_target = False   # for aim acquired/lost logging
    last_move_ts = 0.0   # rate limiter for --move-cooldown

    fps_t0 = time.monotonic()
    fps_count = 0
    fps = 0.0
    frame_count = 0

    from collections import Counter
    debug_t0 = time.monotonic()
    debug_reject_reasons = Counter()
    debug_target_hits = 0
    debug_frame_count = 0

    # OpenCV on QNX is built without GUI support (no GTK / Cocoa).
    # Try imshow once with a 1x1 frame; if it throws, run headless.
    gui_disabled = args.no_preview
    if not gui_disabled:
        try:
            cv2.imshow("__probe__", np.zeros((1, 1, 3), dtype=np.uint8))
            cv2.waitKey(1)
            cv2.destroyWindow("__probe__")
        except cv2.error as e:
            print(f"[detect] cv2.imshow unavailable ({str(e).splitlines()[0][:80]}...) — running headless")
            gui_disabled = True

    aim_target = "dry-run" if args.dry_run else args.serial
    print(f"[detect] {args.width}x{args.height} → {aim_target}. press q to quit.")

    while not stop[0]:
        ok, frame = cap.read()
        if not ok:
            print("[detect] frame grab failed; retrying")
            time.sleep(0.05)
            continue

        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        # Digital zoom: keep the center 1/zoom of the frame in each dimension.
        if args.zoom > 1.0:
            fh0, fw0 = frame.shape[:2]
            cw = max(1, int(fw0 / args.zoom))
            ch = max(1, int(fh0 / args.zoom))
            x0 = (fw0 - cw) // 2
            y0 = (fh0 - ch) // 2
            frame = frame[y0:y0 + ch, x0:x0 + cw]

        ts = time.monotonic()
        h, w = frame.shape[:2]

        # Single grayscale conversion feeds both motion and darkness masks.
        # ~2x faster than running MOG2 on color and loses nothing for this task.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- darkness mask (always) ---
        if args.adaptive:
            # Lighting-invariant: each pixel compared to its local neighborhood
            # instead of a fixed threshold. Handles spotlights, shadows, dim rooms.
            block = args.adaptive_block if args.adaptive_block % 2 == 1 else args.adaptive_block + 1
            dark = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,
                block, args.adaptive_c,
            )
        else:
            _, dark = cv2.threshold(gray, args.dark_threshold, 255, cv2.THRESH_BINARY_INV)

        if args.white_bg:
            # Overfit for a white-background demo: skip MOG2 entirely.
            # Every dark blob is a candidate. Tracking below picks the mover.
            motion = np.zeros_like(dark)     # unused; keep for the show-mask viz
            mask = dark
        else:
            # --- motion mask ---
            motion_raw = bg.apply(gray)
            # MOG2 marks shadows as 127 and true foreground as 255. Drop shadows
            # by keeping only strong foreground pixels before dilating.
            _, motion_fg = cv2.threshold(motion_raw, 200, 255, cv2.THRESH_BINARY)
            # Widen the "where motion happened" region so darkness inside that
            # neighborhood joins the target.
            motion = cv2.dilate(motion_fg, dilate_kernel)
            mask = cv2.bitwise_and(motion, dark)

        # Clean up speckle, then aggressively reconnect fragments.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Collect blobs that pass area + shape filters. Then either pick the
        # biggest (motion-mask mode) or the mover (white-bg mode).
        candidates = []  # list of (contour, area, cx, cy)
        rejected = []    # list of (contour, area, reason)
        for c in contours:
            a = float(cv2.contourArea(c))
            if a < args.min_area:
                rejected.append((c, a, "small"))
                continue
            if a > args.max_area:
                rejected.append((c, a, "big"))
                continue

            # Aspect ratio filter: reject blobs that are too square or too thin.
            (_, _), (bw, bh), _ = cv2.minAreaRect(c)
            longer  = max(bw, bh)
            shorter = max(1.0, min(bw, bh))
            aspect  = longer / shorter
            if aspect < args.aspect_min:
                rejected.append((c, a, f"square {aspect:.1f}"))
                continue
            if aspect > args.aspect_max:
                rejected.append((c, a, f"thin {aspect:.1f}"))
                continue

            # Solidity filter: contour_area / convex_hull_area.
            # Mosquito with spread wings ~0.4-0.6. Phone/rectangle ~1.0.
            hull_a = max(1.0, float(cv2.contourArea(cv2.convexHull(c))))
            solidity = a / hull_a
            if solidity < args.min_solidity:
                rejected.append((c, a, f"solid<{solidity:.2f}"))
                continue
            if solidity > args.max_solidity:
                rejected.append((c, a, f"solid>{solidity:.2f}"))
                continue

            # Extent filter: contour_area / bounding_box_area.
            # Same intent: reject rectangles/blobs that fill their bounding box.
            x_, y_, w_, h_ = cv2.boundingRect(c)
            extent = a / max(1.0, float(w_ * h_))
            if extent < args.min_extent:
                rejected.append((c, a, f"ext<{extent:.2f}"))
                continue
            if extent > args.max_extent:
                rejected.append((c, a, f"ext>{extent:.2f}"))
                continue

            # Compactness — 4πA/P². The mosquito's signature.
            # Smooth blobs (eyebrows, shadows, phones, hands) score 0.5-0.9.
            # Mosquitoes with legs and antennae score 0.05-0.25.
            # This is lighting- and rotation-invariant.
            perimeter = cv2.arcLength(c, True)
            if perimeter < 1.0:
                rejected.append((c, a, "no-perimeter"))
                continue
            compactness = (4.0 * 3.14159 * a) / (perimeter * perimeter)
            if compactness > args.max_compactness:
                rejected.append((c, a, f"smooth {compactness:.2f}"))
                continue

            # Interior mask, reused by fill / brightness / contrast checks.
            blob_mask = np.zeros(gray.shape, dtype=np.uint8)
            cv2.drawContours(blob_mask, [c], -1, 255, thickness=cv2.FILLED)
            interior_pixels = int(cv2.countNonZero(blob_mask))
            if interior_pixels <= 0:
                rejected.append((c, a, "zero-mask"))
                continue

            # Fill-density: how much of the contour's interior is actually dark?
            dark_in_blob = int(cv2.countNonZero(cv2.bitwise_and(dark, blob_mask)))
            fill = dark_in_blob / interior_pixels
            if fill < args.min_fill:
                rejected.append((c, a, f"fill {fill:.2f}"))
                continue

            # Background whiteness: sample a ring around the blob and require
            # MOST of it to be near-white. Mean fails here — an eyebrow's ring
            # is half sheet (bright) + half face (dark), averaging to "ok" while
            # clearly not being on a white background. Fraction-of-white is
            # much stricter and rejects any blob touching a non-sheet region.
            ring_kernel = np.ones((args.bg_ring_px, args.bg_ring_px), np.uint8)
            ring = cv2.subtract(cv2.dilate(blob_mask, ring_kernel), blob_mask)
            ring_size = int(cv2.countNonZero(ring))
            bg_brightness = 0.0
            white_fraction = 0.0
            if ring_size > 0:
                bg_brightness = float(cv2.mean(gray, mask=ring)[0])
                # Fraction of the ring that is genuinely white (above white-cutoff).
                _, white_mask = cv2.threshold(gray, args.white_cutoff, 255, cv2.THRESH_BINARY)
                white_in_ring = int(cv2.countNonZero(cv2.bitwise_and(white_mask, ring)))
                white_fraction = white_in_ring / ring_size
            if white_fraction < args.min_white_fraction:
                rejected.append((c, a, f"nonwhite {white_fraction:.2f}"))
                continue
            if bg_brightness < args.min_bg_brightness:
                rejected.append((c, a, f"bg {int(bg_brightness)}"))
                continue
            # Contrast: blob interior must be MUCH darker than its surroundings.
            blob_brightness = float(cv2.mean(gray, mask=blob_mask)[0])
            contrast = bg_brightness - blob_brightness
            if contrast < args.min_contrast:
                rejected.append((c, a, f"contrast {int(contrast)}"))
                continue

            # Confidence — how strongly did this blob pass every filter?
            # Products of normalized scores so weakness in any dimension hurts.
            c_dark    = 1.0 - blob_brightness / 255.0                  # darker = better
            c_white   = white_fraction                                  # ring whiter = better
            c_contr   = min(1.0, contrast / 200.0)                      # 200+ gap = perfect
            c_fill    = min(1.0, fill / 0.9)                            # solid interior = better
            confidence_val = c_dark * c_white * c_contr * c_fill
            # Bookkeeping — final struct assembled after shape match below.
            _blob_stats = (confidence_val,)

            # Shape template match: reject blobs that don't look like the template.
            # Only bother for blobs big enough to have a stable Hu-moment signature.
            shape_dist = 0.0
            if template_contour is not None and a >= args.shape_min_area:
                shape_dist = cv2.matchShapes(c, template_contour,
                                             cv2.CONTOURS_MATCH_I1, 0.0)
                if shape_dist > args.shape_tolerance:
                    rejected.append((c, a, f"shape {shape_dist:.2f}"))
                    continue
                # Bake the shape score into confidence too.
                confidence_val *= max(0.0, 1.0 - shape_dist / args.shape_tolerance)

            M = cv2.moments(c)
            if M["m00"] <= 0:
                rejected.append((c, a, "zero-moment"))
                continue
            ccx = M["m10"] / M["m00"]
            ccy = M["m01"] / M["m00"]
            candidates.append((c, a, ccx, ccy, confidence_val))

        # ---- Target selection ----
        best = None
        best_area = 0.0
        cx = cy = 0.0
        vx = vy = 0.0
        confidence = 0.0
        detected = False

        if candidates:
            # AIM target = highest-confidence blob. Simple, robust.
            best_conf = -1.0
            for c, a, ccx, ccy, cf in candidates:
                if cf > best_conf:
                    best_conf = cf
                    best = c
                    best_area = a
                    cx, cy = ccx, ccy
                    confidence = cf
            if best_conf >= args.min_confidence:
                detected = True
            else:
                best = None

        # Remember every valid centroid for next frame's white-bg tracking.
        prev_centroids = [(ccx, ccy) for _, _, ccx, ccy, _ in candidates]

        if detected:
            if prev_pt is not None and prev_ts is not None:
                dt = ts - prev_ts
                if dt > 0:
                    vx = (cx - prev_pt[0]) / dt
                    vy = (cy - prev_pt[1]) / dt
            confidence = min(1.0, best_area / args.max_area)
            prev_pt = (cx, cy)
            prev_ts = ts
            lost_frames = 0

            if not had_target:
                print("[aim] target acquired", flush=True)
                had_target = True

            # P-only per axis: output proportional to pixel error from frame
            # center, capped at max_step. Moves are also rate-limited by
            # --move-cooldown so the servos step gently instead of jittering
            # at frame rate.
            frame_cx, frame_cy = w / 2.0, h / 2.0
            ex = cx - frame_cx
            ey = cy - frame_cy
            pan_step = clamp(args.kp_pan * ex, -args.max_step, args.max_step)
            tilt_step = clamp(args.kp_tilt * ey, -args.max_step, args.max_step)

            if ts - last_move_ts >= args.move_cooldown:
                moved = []
                if abs(pan_step) >= 1:
                    sink.send(f"PAN:{int(round(pan_step))}")
                    moved.append(f"PAN:{int(round(pan_step))}")
                if abs(tilt_step) >= 1:
                    sink.send(f"TILT:{int(round(tilt_step))}")
                    moved.append(f"TILT:{int(round(tilt_step))}")

                if moved:
                    last_move_ts = ts
                    print(f"[aim] move sent: {' '.join(moved)}  "
                          f"(ex={ex:+.1f} ey={ey:+.1f})", flush=True)
                else:
                    print(f"[aim] no move — within deadband "
                          f"(ex={ex:+.1f} ey={ey:+.1f})", flush=True)
        else:
            lost_frames += 1
            if lost_frames > 5:
                prev_pt = None
                prev_ts = None
            if had_target:
                print("[aim] target lost", flush=True)
                had_target = False

        frame_count += 1

        # --- FPS counter ---
        fps_count += 1
        if ts - fps_t0 >= 1.0:
            fps = fps_count / (ts - fps_t0)
            fps_count = 0
            fps_t0 = ts

        # --- headless debug stats ---
        if args.debug:
            debug_frame_count += 1
            if detected:
                debug_target_hits += 1
            for _c, _a, reason in rejected:
                # Group by first word so "solid>0.72" collapses to "solid>".
                key = reason.split()[0] if reason else "unknown"
                debug_reject_reasons[key] += 1
            if ts - debug_t0 >= 2.0:
                total_rej = sum(debug_reject_reasons.values())
                top = ", ".join(
                    f"{k}={v}" for k, v in debug_reject_reasons.most_common(6))
                import sys
                print(f"[detect] frames={debug_frame_count} "
                      f"targets={debug_target_hits} rej={total_rej}  {top}",
                      file=sys.stderr, flush=True)
                debug_frame_count = 0
                debug_target_hits = 0
                debug_reject_reasons.clear()
                debug_t0 = ts

        # Draw overlays if we're going to either display or save the frame.
        should_save_debug = args.save_debug > 0 and (frame_count % args.save_debug == 0)
        should_stream = mjpeg is not None
        should_draw = (not args.no_preview and not gui_disabled) or should_save_debug or should_stream
        if should_draw:
            # Draw rejected blobs in yellow with the reason so we can tune.
            for c, a, reason in rejected:
                x, y, ww, hh = cv2.boundingRect(c)
                color = (0, 165, 255)  # orange-ish yellow
                cv2.rectangle(frame, (x, y), (x + ww, y + hh), color, 1)
                cv2.putText(frame, f"{reason} a={int(a)}", (x, y - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Draw every mosquito. AIM (highest conf) = blue. Others = green.
            for c, a, ccx, ccy, cf in candidates:
                x, y, ww, hh = cv2.boundingRect(c)
                is_aim = (best is not None) and (c is best)
                if is_aim:
                    # Blue box + crosshair + velocity vector for the aim target.
                    cv2.rectangle(frame, (x, y), (x + ww, y + hh), (255, 128, 0), 3)
                    cv2.drawMarker(frame, (int(ccx), int(ccy)), (0, 0, 255),
                                   cv2.MARKER_CROSS, 20, 2)
                    end = (int(ccx + vx * 0.12), int(ccy + vy * 0.12))
                    cv2.arrowedLine(frame, (int(ccx), int(ccy)), end,
                                    (0, 255, 255), 2, tipLength=0.3)
                    cv2.putText(frame, f"AIM c={cf:.2f}", (x, y - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 128, 0), 2)
                else:
                    cv2.rectangle(frame, (x, y), (x + ww, y + hh), (0, 255, 0), 1)
                    cv2.putText(frame, f"c={cf:.2f}", (x, y - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            template_status = "TEMPLATE ON" if template_contour is not None else "no template (T to capture)"
            mode = "WHITE-BG" if args.white_bg else "motion"
            cv2.putText(frame,
                        f"fps={fps:4.1f}  targets={len(candidates)}  "
                        f"rej={len(rejected)}  [{mode}]  [{template_status}]",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)
            cv2.putText(frame, "T=capture template  C=clear  Q=quit",
                        (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (200, 200, 200), 1)

            # Compose mask visualization once — shared by show and save paths.
            mask_viz = None
            if args.show_mask or should_save_debug:
                colored = np.zeros_like(frame)
                colored[..., 2] = motion              # R = motion
                colored[..., 0] = dark                # B = dark
                combined3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                mask_viz = cv2.addWeighted(colored, 0.7, combined3, 0.5, 0)


            if should_save_debug:
                try:
                    cv2.imwrite("/tmp/buzzkill_frame.png", frame)
                    if mask_viz is not None:
                        cv2.imwrite("/tmp/buzzkill_mask.png", mask_viz)
                except Exception as e:
                    print(f"[detect] imwrite failed: {e}")

            if mjpeg is not None:
                mjpeg.publish(frame)

            if not args.no_preview and not gui_disabled:
                cv2.imshow("buzzkill detect", frame)
                if args.show_mask and mask_viz is not None:
                    cv2.imshow("mask (R=motion B=dark W=both)", mask_viz)

                key = cv2.waitKey(1) & 0xFF
                if key == 0xFF:  # nothing pressed
                    pass
                elif key == ord("q"):
                    break
                elif key == ord("t") and best is not None:
                    template_contour = best.copy()
                    try:
                        np.save(args.template, template_contour, allow_pickle=False)
                        print(f"[detect] template captured "
                              f"({len(template_contour)} points) → {args.template}")
                    except Exception as e:
                        print(f"[detect] template save failed: {e}")
                elif key == ord("c"):
                    template_contour = None
                    print("[detect] template cleared — matching disabled")

    sink.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
