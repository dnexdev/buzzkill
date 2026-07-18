"""Motion detection pipeline. Sends target packets over UDP.

Usage:
  python detect.py --source 0                       # webcam
  python detect.py --source http://esp32.local/mjpeg
  python detect.py --source 0 --target 192.168.1.42:9000
"""
import argparse
import socket
import time

import cv2
import numpy as np

from protocol import make_packet, encode


def parse_hostport(s):
    host, port = s.rsplit(":", 1)
    return host, int(port)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="0 for webcam, or URL")
    ap.add_argument("--target", default="127.0.0.1:9000")
    ap.add_argument("--min-area", type=int, default=80,
                    help="min contour area in pixels")
    ap.add_argument("--max-area", type=int, default=8000)
    ap.add_argument("--history", type=int, default=200,
                    help="MOG2 history frames")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {args.source}")

    host, port = parse_hostport(args.target)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    bg = cv2.createBackgroundSubtractorMOG2(
        history=args.history, varThreshold=32, detectShadows=False)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    prev_pt = None
    prev_ts = None
    lost_frames = 0

    print(f"streaming to {host}:{port}. press q to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("frame grab failed")
            break
        ts = time.time()
        h, w = frame.shape[:2]

        mask = bg.apply(frame)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0
        for c in contours:
            a = cv2.contourArea(c)
            if a < args.min_area or a > args.max_area:
                continue
            if a > best_area:
                best_area = a
                best = c

        detected = best is not None
        if detected:
            M = cv2.moments(best)
            if M["m00"] == 0:
                detected = False
            else:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]

        if detected:
            if prev_pt is not None and prev_ts is not None:
                dt = ts - prev_ts
                if dt > 0:
                    vx = (cx - prev_pt[0]) / dt
                    vy = (cy - prev_pt[1]) / dt
                else:
                    vx = vy = 0.0
            else:
                vx = vy = 0.0

            confidence = min(1.0, best_area / args.max_area)
            pkt = make_packet(w, h, cx, cy, vx, vy, confidence, True)
            prev_pt = (cx, cy)
            prev_ts = ts
            lost_frames = 0
        else:
            lost_frames += 1
            if lost_frames > 5:
                prev_pt = None
                prev_ts = None
            pkt = make_packet(w, h, -1, -1, 0, 0, 0, False)

        sock.sendto(encode(pkt), (host, port))

        if not args.no_preview:
            if detected:
                x, y, ww, hh = cv2.boundingRect(best)
                cv2.rectangle(frame, (x, y), (x + ww, y + hh), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)),
                           4, (0, 0, 255), -1)
                cv2.putText(frame, f"a={int(best_area)}", (x, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("buzzkill detect", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
