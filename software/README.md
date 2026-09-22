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
  R 0.3098, L 0.3226, U 0.2106, D 0.1200) so a full natural look hits those host limits

Lids follow the host (`left_open` / `right_open` in `[0,1]` from webcam EAR) —
no firmware auto-blink.
On boot firmware **assumes** the mechanism is already at HOME (does not drive there).

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
draws white periocular orbit + red iris circles + faint green eyelid contour
(visual only). For each eye, pupil center vs orbit center uses the same
``θ = s / r`` math as the mouse pad; L/R are averaged, scaled by calibrated
iris extremes to host degrees (see below), then clamped to ±14.0° X / ±4.5° Y
and sent to the Arduino with independent L/R lid openness from EAR.

```bash
# Preview / drive — Connect from the control window, then Start for iris follow
python pupil_control.py
python pupil_control.py --port /dev/ttyUSB0   # optional auto-connect
python pupil_control.py --port COM5 --invert-y

# Print protocol lines instead of serial
python pupil_control.py --mock

python pupil_control.py --list-ports
python pupil_control.py --image test_face.jpg
```

Flow: **Connect → arrows-only + lids follow webcam → Start → iris follow.**
`q` in the webcam window quits. No face for > `--lose-timeout` frames while
tracking recenters gaze to `0,0` with lids open. Lids are EAR-driven
(`EAR_CLOSED=0.15` → closed, `EAR_OPEN=0.25` → open, EMA `LID_SMOOTH=0.45`)
plus a down-gaze bias (up to +0.35 at full down) so looking down isn't read
as a blink.

| Flag | Default | Meaning |
|------|---------|---------|
| `--port` | none | Serial device; omit = preview only |
| `--mock` | off | Write `right_deg,up_deg,left_open,right_open` to stdout |
| `--baud` | 9600 | Must match firmware |
| `--gain` | 1.0 | Amplify iris offsets |
| `--invert-y` | off | Flip vertical if mounting is mirrored |
| `--lose-timeout` | 15 | Frames without face before sending `0,0` |
| `--camera` | 0 | Webcam index |
| `--width` / `--height` | 1280 / 720 | Capture size (0 = leave default) |
| `--image` / `--out` | none | Still-image test instead of webcam |
| `--model` | `models/face_landmarker.task` | Auto-downloaded if missing |

Host limits are constants (`MAX_TURN_X=14.0`, `MAX_TURN_Y=4.5`), not flags —
they come from `calibrate.py`. Iris θ extremes are constants
(`R 0.3098 / L 0.3226 / U 0.2106 / D 0.1200` from `eye_calib_limits.txt`):
full natural look in each direction maps to the host limits above.

## Firmware

Flash [`firmware/firmware.ino`](firmware/firmware.ino). Pins D3/D5
(X, Y) + D6/D9/D10/D11 (four blink lids). See root [`README.md`](../README.md) for the table.
