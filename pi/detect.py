"""Motion + darkness detection pipeline. Sends target packets over UDP.

Optimized for the demo: black paper mosquitoes swinging on strings against
a lighter background. The pipeline is:

  frame -> MOG2 motion mask -> AND -> darkness mask -> morph -> contours -> centroid

The AND is the key trick: motion alone triggers on hands and walking people;
darkness alone triggers on any dark object (a chair leg). Together they only
fire when something dark is moving.

Usage:
  python3 detect.py --source 0
  python3 detect.py --source 0 --target 192.168.1.42:9000
  python3 detect.py --source 0 --show-mask       # tune the darkness threshold
"""
from __future__ import annotations

import argparse
import socket
import time

import cv2
import numpy as np

from protocol import make_packet, encode


def parse_hostport(s: str):
    host, port = s.rsplit(":", 1)
    return host, int(port)


def open_camera(source, width, height):
    src = int(source) if str(source).isdigit() else source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {source}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Try to get low latency — some backends honor this, some don't.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # Warmup — first frames from CSI cameras are often garbage.
    for _ in range(5):
        cap.read()
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0",
                    help="0 for CSI/webcam, or URL")
    ap.add_argument("--target", default="127.0.0.1:9000",
                    help="host:port of the controller (main.py)")
    ap.add_argument("--width",  type=int, default=640)
    ap.add_argument("--height", type=int, default=480)

    ap.add_argument("--dark-threshold", type=int, default=80,
                    help="pixel < this counts as dark (0-255)")
    ap.add_argument("--min-area", type=int, default=60,
                    help="min contour area in pixels")
    ap.add_argument("--max-area", type=int, default=1500,
                    help="max contour area — tighten to reject people/clothes")
    ap.add_argument("--history", type=int, default=200,
                    help="MOG2 background history frames")
    ap.add_argument("--var-threshold", type=int, default=32)
    ap.add_argument("--motion-dilate", type=int, default=15,
                    help="dilate motion mask by N px before ANDing with darkness. "
                         "prevents fragmentation when only part of the target moved.")
    ap.add_argument("--close-kernel", type=int, default=11,
                    help="morph-close kernel size (px). bigger = reconnects fragments harder.")

    # Shape filters — the "overfit to our demo mosquito" knobs.
    ap.add_argument("--aspect-min", type=float, default=0.0,
                    help="min longer/shorter side ratio; 1.5 rejects square blobs")
    ap.add_argument("--aspect-max", type=float, default=99.0,
                    help="max longer/shorter side ratio; caps ultra-thin blobs")
    ap.add_argument("--min-solidity", type=float, default=0.0,
                    help="area / convex-hull area. mosquito ~0.4-0.6, phone ~1.0. set 0.3 to reject phones/shirts")
    ap.add_argument("--max-solidity", type=float, default=1.0,
                    help="upper bound on solidity. set 0.85 to reject any near-rectangle")
    ap.add_argument("--min-extent", type=float, default=0.0,
                    help="area / bounding-box area. mosquito ~0.4, phone ~1.0")
    ap.add_argument("--max-extent", type=float, default=1.0,
                    help="upper bound on extent. set 0.85 to reject any near-rectangle")
    ap.add_argument("--template", default="template.npy",
                    help="path to saved shape template (auto-loaded if present)")
    ap.add_argument("--shape-tolerance", type=float, default=0.5,
                    help="max matchShapes distance vs template; lower = stricter")

    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--show-mask", action="store_true",
                    help="show combined detection mask alongside frame")
    args = ap.parse_args()

    cap = open_camera(args.source, args.width, args.height)
    host, port = parse_hostport(args.target)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
        detectShadows=False,
    )
    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                             (args.close_kernel, args.close_kernel))
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                              (args.motion_dilate, args.motion_dilate))

    prev_pt = None
    prev_ts = None
    lost_frames = 0

    fps_t0 = time.monotonic()
    fps_count = 0
    fps = 0.0
    pkt_count = 0

    print(f"[detect] {args.width}x{args.height} → udp {host}:{port}. press q to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[detect] frame grab failed; retrying")
            time.sleep(0.05)
            continue
        ts = time.monotonic()
        h, w = frame.shape[:2]

        # Single grayscale conversion feeds both motion and darkness masks.
        # ~2x faster than running MOG2 on color and loses nothing for this task.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # --- motion mask ---
        motion_raw = bg.apply(gray)
        # Widen the "where motion happened" region so darkness inside that
        # neighborhood joins the target. Without this, a swinging mosquito's
        # near-stationary body is dropped and we only see the wing tip.
        motion = cv2.dilate(motion_raw, dilate_kernel)

        # --- darkness mask ---
        _, dark = cv2.threshold(gray, args.dark_threshold, 255, cv2.THRESH_BINARY_INV)

        # --- combined mask: (roughly moving) AND dark ---
        mask = cv2.bitwise_and(motion, dark)

        # Clean up speckle, then aggressively reconnect fragments.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Pick the biggest blob within area bounds AND matching shape constraints.
        # Keep the rejected ones around so we can draw them in the preview for tuning.
        best = None
        best_area = 0.0
        rejected = []  # list of (contour, area, reason)
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

            # Shape template match: reject blobs that don't look like the template.
            if template_contour is not None:
                # matchShapes uses Hu moments — scale + rotation invariant.
                dist = cv2.matchShapes(c, template_contour,
                                       cv2.CONTOURS_MATCH_I1, 0.0)
                if dist > args.shape_tolerance:
                    rejected.append((c, a, f"shape {dist:.2f}"))
                    continue

            if a > best_area:
                best_area = a
                best = c

        cx = cy = 0.0
        vx = vy = 0.0
        confidence = 0.0
        detected = False

        if best is not None:
            M = cv2.moments(best)
            if M["m00"] > 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                detected = True

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
            pkt = make_packet(w, h, cx, cy, vx, vy, confidence, True)
        else:
            lost_frames += 1
            if lost_frames > 5:
                prev_pt = None
                prev_ts = None
            pkt = make_packet(w, h, -1, -1, 0, 0, 0, False)

        sock.sendto(encode(pkt), (host, port))
        pkt_count += 1

        # --- FPS counter ---
        fps_count += 1
        if ts - fps_t0 >= 1.0:
            fps = fps_count / (ts - fps_t0)
            fps_count = 0
            fps_t0 = ts

        if not args.no_preview:
            # Draw rejected blobs in yellow with the reason so we can tune.
            for c, a, reason in rejected:
                x, y, ww, hh = cv2.boundingRect(c)
                color = (0, 165, 255)  # orange-ish yellow
                cv2.rectangle(frame, (x, y), (x + ww, y + hh), color, 1)
                cv2.putText(frame, f"{reason} a={int(a)}", (x, y - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            if detected:
                x, y, ww, hh = cv2.boundingRect(best)
                cv2.rectangle(frame, (x, y), (x + ww, y + hh), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                # Velocity vector — shows where we're leading toward.
                end = (int(cx + vx * 0.12), int(cy + vy * 0.12))
                cv2.arrowedLine(frame, (int(cx), int(cy)), end,
                                (255, 255, 0), 2, tipLength=0.3)
                cv2.putText(frame,
                            f"TARGET a={int(best_area)} conf={confidence:.2f}",
                            (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1)

            template_status = "TEMPLATE ON" if template_contour is not None else "no template (T to capture)"
            cv2.putText(frame,
                        f"fps={fps:4.1f}  pkts={pkt_count}  "
                        f"dark<{args.dark_threshold}  area=[{args.min_area},{args.max_area}]  "
                        f"rej={len(rejected)}  [{template_status}]",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)
            cv2.putText(frame, "T=capture template  C=clear  Q=quit",
                        (10, args.height - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (200, 200, 200), 1)

            cv2.imshow("buzzkill detect", frame)

            if args.show_mask:
                # Colorize: red = motion only, blue = dark only, white = both (target).
                colored = np.zeros_like(frame)
                colored[..., 2] = motion              # R channel = motion
                colored[..., 0] = dark                # B channel = dark
                combined3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                cv2.imshow("mask (R=motion B=dark W=both)",
                           cv2.addWeighted(colored, 0.7, combined3, 0.5, 0))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("t") and best is not None:
                # Capture the currently-locked target's contour as the template.
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

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
