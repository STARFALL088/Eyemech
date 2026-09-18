# Eyemech — computer-side software

Two **supported** ways to drive `firmware/firmware.ino` over USB-serial
(9600 baud):

| Script | Role |
|--------|------|
| `direct_control.py` | Tkinter mouse pad → degrees → Arduino |
| `pupil_control.py` | Webcam + MediaPipe iris → same degrees → Arduino |
| `calibrate.py` | Arrow-only; records max L/R/U/D **host degrees** (Arduino) |
| `eye_calibrate.py` | Webcam only; records max iris **θ = s/r** (unit disk) |

**Protocol**
- Gaze: `"<right_deg>,<up_deg>\n"` — `0,0` = look straight (calibrated midpoints)
- Park: `"HOME\n"` — calibrated zero pose (sent on clean Disconnect / app exit)
- Host angles are clamped per axis from mechanism calibration: **±14° X**, **±4.5° Y**,
  then mapped onto each eyeball’s calibrated min…max (X: 0–80, Y: 60–120)
- `pupil_control.py` scales iris θ by measured extremes (see `eye_calib_limits.txt`:
  R≈0.34, L≈0.52, U≈0.21) so a full natural look hits those host limits

Firmware auto-blinks eyelids using calibrated open/closed endpoints.
On boot it **assumes** the mechanism is already at HOME (does not drive there).

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

## Range calibrator

```bash
python calibrate.py
```

Arrow-only UI (no mouse pad). On start it creates `calib_limits_YYYYMMDD_HHMMSS.txt`.
**Home** sets the current pose as zero and **resets** recorded extremes.
Then nudge ← → ↑ ↓ to safe edges; `max_right` / `max_left` / `max_up` / `max_down`
update whenever you beat a relative extreme. Quit and share that `.txt`.

## Iris range calibrator (no Arduino)

```bash
python eye_calibrate.py
```

Webcam + MediaPipe only. Look hard left/right/up/down; peaks of unit-disk
`θ = s/r` are written to `eye_calib_limits_*.txt`. Share that file so we can
scale `pupil_control` mapping to your real iris travel.

## MediaPipe iris / pupil

Tracks iris center inside each eye (reliable realtime proxy for “pupil”),
draws green eye circles + red iris circles. For each eye, pupil center vs
eye-circle center uses the same ``θ = s / r`` math as the mouse pad; L/R
are averaged, printed to the terminal (rad + deg), then clamped to
±`--max-turn` and sent to the Arduino.

```bash
# Preview / drive — Connect from the control window
python pupil_control.py
python pupil_control.py --port /dev/ttyUSB0   # optional auto-connect
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
