# Eyemech — computer-side software

Two **supported** ways to drive `firmware/firmware.ino` over USB-serial
(9600 baud, `"right_deg,up_deg\n"`, center = `0,0`):

| Script | Role |
|--------|------|
| `direct_control.py` | Tkinter mouse pad → degrees → Arduino |
| `pupil_control.py` | Webcam + MediaPipe iris → same degrees → Arduino |

Firmware clamps with `safe_turn()` to ±`MAX_TURN_ANGLE` (15°). Four eyelid
servos blink on a timer; X/Y follow the last serial command.

## Install

```bash
cd software
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
# Live OpenCV windows (recommended on a desktop):
pip install "opencv-python>=4.7,<5"
# Mouse pad UI (Linux):
sudo apt install python3-tk
```

`pupil_control.py` auto-downloads `models/face_landmarker.task` on first run.

## Direct mouse pad

```bash
python direct_control.py
```

Pick the serial port, click **Connect**, move the cursor inside the circle.
Angles print in the terminal and stream to the board when connected.

## MediaPipe iris / pupil

Tracks iris center inside each eye (reliable realtime proxy for “pupil”),
draws green eye circles + red iris circles. For each eye, pupil center vs
eye-circle center uses the same ``θ = s / r`` math as the mouse pad; L/R
are averaged, printed to the terminal (rad + deg), then clamped to
±`--max-turn` and sent to the Arduino.

```bash
# Preview only (no Arduino)
python pupil_control.py

# Drive firmware
python pupil_control.py --port /dev/ttyUSB0
python pupil_control.py --port COM5 --invert-y

# Print protocol lines instead of serial
python pupil_control.py --mock

python pupil_control.py --list-ports
python pupil_control.py --image test_face.jpg
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--port` | none | Serial device; omit = preview only |
| `--mock` | off | Write `right_deg,up_deg` to stdout |
| `--baud` | 9600 | Must match firmware |
| `--max-turn` | 15 | Degrees at full iris offset |
| `--gain` | 1.0 | Amplify iris offsets |
| `--invert-y` | off | Flip vertical if mounting is mirrored |
| `--lose-timeout` | 15 | Frames without face before sending `0,0` |
| `--camera` | 0 | Webcam index |

## Firmware

Flash [`firmware/firmware.ino`](firmware/firmware.ino). Named pins D2–D7
(X, Y, four blink lids). See root [`README.md`](../README.md) for the table.

## Legacy / experimental

These remain in the tree but are **not** the preferred path for the new firmware:

| File | Notes |
|------|--------|
| `eyemech_gaze.py` | OpenCV Haar face/pupil → old `0..1023` protocol for `Basic movements for openCV.ino` (PCA9685) |
| `eye_track_basic.py` | Minimal Haar + blob pupil demo |
| `gaze_analysis.py` | Offline fixation / dyslexia-style metrics (no camera) |
| `verify_firmware_compat.py` | Checks old Python↔PCA9685 mapping |

Haar/blob live tracking is fragile; prefer `pupil_control.py` (MediaPipe) or
`direct_control.py` (mouse).
