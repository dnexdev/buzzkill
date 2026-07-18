# buzzkill

Nerf turret that shoots swinging paper mosquitoes. Hack the 6ix 2026.

## Architecture

```
[ESP32-CAM] --MJPEG--> [Laptop: OpenCV detect] --UDP JSON--> [RP5/QNX: aim + fire] --PWM--> [servos + flywheel]
```

- **Laptop** (`laptop/`) — Python + OpenCV. Motion detection, sends target position and velocity per frame.
- **QNX** (`qnx/`) — C++. Receives targets, predicts lead, drives servos, decides fire.

The laptop only sees pixels. The QNX box owns the world model, servo timing, and safety.

## Quick start

### Laptop (dev with webcam)
```
cd laptop
pip install -r requirements.txt
python detect.py --source 0 --target 127.0.0.1:9000
```

### QNX side (dev on macOS with mock servo)
```
cd qnx
make
./build/buzzkill --mock --port 9000
```

Press Ctrl+C to stop. Mock servo prints commands to stdout.

### Calibration
```
python laptop/calibrate.py --source 0 --out qnx/config/calibration.json
```
Aim turret at 4 corner points, click each in the frame, press ENTER. Saved map is loaded by QNX at boot.

## Protocol
See `laptop/protocol.py` and `qnx/src/protocol.h`. JSON over UDP so you can `nc -u -l 9000` for debugging.
