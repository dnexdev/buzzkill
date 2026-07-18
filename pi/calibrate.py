"""4-corner calibration. Aim the turret at each corner, click where it points,
enter the pan/tilt angles you dialed in. Saves a JSON map for QNX to load.

Usage:
  python calibrate.py --source 0 --out ../qnx/config/calibration.json
"""
import argparse
import json

import cv2

CORNERS = [
    ("top-left",     0, 0),
    ("top-right",    1, 0),
    ("bottom-right", 1, 1),
    ("bottom-left",  0, 1),
]


def prompt_angles(name):
    while True:
        s = input(f"{name}: enter pan,tilt degrees (e.g. -20,15): ").strip()
        try:
            pan, tilt = s.split(",")
            return float(pan), float(tilt)
        except Exception:
            print("bad input, try again")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {args.source}")

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("could not grab initial frame")
    h, w = frame.shape[:2]

    points = []
    clicked = {"pt": None}

    def on_click(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked["pt"] = (x, y)

    cv2.namedWindow("calibrate")
    cv2.setMouseCallback("calibrate", on_click)

    for name, _, _ in CORNERS:
        print(f"\n=== aim turret at {name} of scene, then click that spot ===")
        clicked["pt"] = None
        while clicked["pt"] is None:
            ok, frame = cap.read()
            if not ok:
                continue
            for p in points:
                cv2.circle(frame, (int(p["px"]), int(p["py"])),
                           6, (0, 255, 0), 2)
            cv2.putText(frame, f"click {name}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("calibrate", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                raise SystemExit("aborted")
        px, py = clicked["pt"]
        pan, tilt = prompt_angles(name)
        points.append({"name": name, "px": px, "py": py,
                       "pan": pan, "tilt": tilt})
        print(f"  recorded: px={px} py={py} pan={pan} tilt={tilt}")

    cap.release()
    cv2.destroyAllWindows()

    out = {"frame_w": w, "frame_h": h, "points": points}
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
