# Eyemech — Python / OpenCV Gaze Controller

This is the **computer side** of Eyemech. The Arduino firmware
(`../Basic movements for openCV.ino`) drives the 6-servo eye mechanism over a
PCA9685 PWM board. It expects gaze coordinates over Serial as ASCII
`"x,y\n"` lines (each value `0..1023`, `512,512` = centered).

This script opens a webcam, finds your face (and optionally your pupils), and
streams that same protocol to the Arduino. Together the two files make the
eye look around and follow you.

## Install

```bash
pip install -r requirements.txt
```

(On a headless box you can run `--selftest` / `--mock` without a camera;
`cv2` is only needed for live tracking.)

## Hardware wiring assumptions

The firmware uses:

* PCA9685 on I2C (Adafruit_PWMServoDriver, default address)
* Servo channels 0–5: `0,1` = left/right eyeball X/Y, `2–5` = eyelids
* Trim pot on `A2`, override button on pin `2`
* Serial @ 9600 baud

Match these in `software` if you change them on the Arduino side.

## Usage

```bash
# Face-follow mode, real serial port:
python3 eyemech_gaze.py --port /dev/ttyUSB0

# Auto-pick the only connected port:
python3 eyemech_gaze.py            # (prompts if >1 port)

# Headless test — prints protocol lines instead of writing serial:
python3 eyemech_gaze.py --mock --preview

# True-gaze (pupil) mode, flip vertical for your servo mounting:
python3 eyemech_gaze.py --port /dev/ttyACM0 --mode pupil --invert-y

# See all flags:
python3 eyemech_gaze.py --help

# Verify the coordinate math without any hardware:
python3 eyemech_gaze.py --selftest
```

### Common flags

| Flag | Default | Meaning |
|------|---------|---------|
| `--port` | auto | Serial port (`/dev/ttyUSB0`, `COM3`, …). Omit + `--mock` for stdout. |
| `--baud` | 9600 | Must match firmware. |
| `--mock` | off | Print `x,y\n` to stdout instead of serial (no Arduino needed). |
| `--mode` | `face` | `face` = head-position follow; `pupil` = true gaze from pupil offset. |
| `--camera` | 0 | Camera index. |
| `--track-range` | 0.6 | Face-follow sensitivity (smaller = more sensitive). |
| `--pupil-gain` | 1.0 | Amplify pupil offsets in `pupil` mode. |
| `--invert-y` | off | Flip vertical axis to match mirrored servo mounting. |
| `--smooth` | 0.0 | EMA smoothing alpha (firmware already smooths; keep small). |
| `--preview` | off | Show the webcam window. |
| `--verbose` | off | Log each computed gaze value. |

## Coordinate protocol (must match the firmware)

* One line per processed frame: `"<x>,<y>\n"`
* `x`, `y` are integers constrained to `[0, 1023]`
* `512,512` = eyes centered / looking straight ahead
* The frame is mirrored (`cv2.flip`) so motion feels like looking in a mirror
* If the face is lost, the eyes hold the last good position, then slowly ease
  back to center after `--lose-timeout` frames

The mapping math lives in pure functions (`norm_to_servo`, `face_to_norm`,
`pupil_offset_to_servo`) so it can be unit-tested with `--selftest` without a
camera.

## Tips

* **`face` mode is the reliable default** — it tracks your head position, so the
  eyes follow you around the room. Works in most lighting.
* **`pupil` mode** is cooler but needs good light and a close camera; tune
  `--pupil-gain` and `--deadzone` for your setup.
* If the robot eye looks *down* when you look *up*, add `--invert-y`.
* The firmware already does a 5-sample rolling average — you usually don't need
  extra `--smooth`.
