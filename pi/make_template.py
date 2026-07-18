"""Extract a shape template from a reference image and save it as template.npy.

Usage:
  # save the mosquito image to pi/mosquito.png, then:
  python3 make_template.py --image mosquito.png
  python3 make_template.py --image mosquito.png --threshold 100 --preview

The output template.npy is loaded automatically by detect.py.
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="path to reference image (mosquito silhouette)")
    ap.add_argument("--out", default="template.npy")
    ap.add_argument("--threshold", type=int, default=100,
                    help="pixel < this counts as part of the silhouette (0-255)")
    ap.add_argument("--preview", action="store_true",
                    help="show the extracted contour before saving")
    args = ap.parse_args()

    img = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"cannot read {args.image}")

    _, mask = cv2.threshold(img, args.threshold, 255, cv2.THRESH_BINARY_INV)

    # Fill small internal holes so the contour is one clean outline.
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise SystemExit(f"no dark contours found in {args.image} — try --threshold higher")
    biggest = max(contours, key=cv2.contourArea)

    # Compute the same shape stats detect.py will use, so you can tune from here.
    a = float(cv2.contourArea(biggest))
    hull = cv2.convexHull(biggest)
    hull_a = max(1.0, float(cv2.contourArea(hull)))
    solidity = a / hull_a
    x, y, w, h = cv2.boundingRect(biggest)
    extent = a / max(1.0, float(w * h))
    (_, _), (bw, bh), _ = cv2.minAreaRect(biggest)
    aspect = max(bw, bh) / max(1.0, min(bw, bh))

    print(f"[template] points   : {len(biggest)}")
    print(f"[template] area     : {int(a)} px")
    print(f"[template] aspect   : {aspect:.2f}")
    print(f"[template] solidity : {solidity:.2f}   (mosquito ~0.4-0.6, phone/rect ~1.0)")
    print(f"[template] extent   : {extent:.2f}   (mosquito ~0.4-0.5, phone/rect ~1.0)")

    np.save(args.out, biggest, allow_pickle=False)
    print(f"[template] wrote {args.out}")

    if args.preview:
        disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(disp, [biggest], -1, (0, 255, 0), 2)
        cv2.drawContours(disp, [hull],    -1, (0, 128, 255), 1)
        cv2.rectangle(disp, (x, y), (x + w, y + h), (255, 0, 0), 1)
        cv2.imshow("template (green=contour, orange=hull, blue=bbox)", disp)
        print("press any key to close")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
