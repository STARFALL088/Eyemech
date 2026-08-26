#!/usr/bin/env python3
"""
Eyemech — OpenCV Gaze Control (Python side)
===========================================

Companion script to ``Basic movements for openCV.ino``.

The Arduino firmware sits on an Adafruit PCA9685 PWM servo board driving a 6-servo
animatronic eye (2 eyeball X/Y + 4 eyelids). It reads gaze coordinates over Serial
as ASCII lines ``"x,y\\n"`` where x and y are integers in [0, 1023].

This script is the OTHER half of the system: it opens a webcam, locates the user's
face (and optionally pupils), and streams the matching ``x,y\\n`` protocol to the
Arduino over a USB-serial port.

Two tracking modes
-------------------
``face``   (default): the robotic eyes track the *position of your head* in the
           camera frame — walk left and the eyes swing left, lean in and they
           look "down" toward you. This is the classic, rock-solid "the eyes
           follow you around the room" effect and needs no precise pupil lock.

``pupil``  : true gaze estimation. Within each detected eye we find the pupil
           centroid and report its offset from eye-center, so the robot eye looks
           the direction you are actually looking. More fragile (needs decent
           lighting / close camera) but cooler when it locks.

Pure math (mapping) is isolated in standalone functions so it can be unit-tested
without a camera. Run ``python3 eyemech_gaze.py --selftest`` to verify.

Usage
-----
  # Real hardware, auto-pick serial port, face-follow mode:
  python3 eyemech_gaze.py --port /dev/ttyUSB0

  # Headless test — print protocol lines instead of writing serial:
  python3 eyemech_gaze.py --mock

  # Pupil-gaze mode, flip Y to match your servo mounting:
  python3 eyemech_gaze.py --port /dev/ttyACM0 --mode pupil --invert-y

  # List available serial ports and exit:
  python3 eyemech_gaze.py --list-ports

Protocol (must match firmware)
------------------------------
  * One line per frame:  "<x>,<y>\\n"
  * x, y are integers constrained to [0, 1023]
  * 512,512 == centered (eyes straight ahead)
  * Little-to-no smoothing on the wire; the firmware already does a 5-sample
    rolling average. Extra Python-side EMA is optional (--smooth).

Author: STARFALL088
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

# --- OpenCV / numpy (imported lazily where possible so --selftest is light) ---
try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None  # type: ignore

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore


# =============================================================================
# Pure mapping math  (no camera / no serial required — unit-testable)
# =============================================================================

SERVO_MIN, SERVO_MAX = 0, 1023
CENTER = 512  # firmware defaults xval/yval to 512 — keep in lockstep


def clamp_servo(v: float) -> int:
    """Clamp any value into the firmware's valid [0, 1023] range."""
    return int(max(SERVO_MIN, min(SERVO_MAX, round(v))))


def norm_to_servo(norm: float) -> int:
    """
    Convert a normalized coordinate in [0.0, 1.0] (0.5 == center) to a
    firmware servo value in [0, 1023].

    norm 0.0 -> 0   (look hard one way)
    norm 0.5 -> 512 (centered)
    norm 1.0 -> 1023 (look hard the other way)
    """
    n = max(0.0, min(1.0, norm))
    return clamp_servo(n * (SERVO_MAX - SERVO_MIN) + SERVO_MIN)


def apply_deadzone(offset: float, deadzone: float) -> float:
    """
    Kill tiny jitter around center. ``offset`` in [-1, 1]; ``deadzone`` in [0, 1].
    Returns a (possibly scaled-up) offset in [-1, 1].
    """
    if abs(offset) <= deadzone:
        return 0.0
    # Rescale the surviving range so motion stays full-scale outside the deadzone.
    sign = 1.0 if offset > 0 else -1.0
    return sign * (abs(offset) - deadzone) / (1.0 - deadzone)


def face_to_norm(
    face_cx: float,
    face_cy: float,
    frame_w: int,
    frame_h: int,
    track_range: float = 0.6,
    deadzone: float = 0.04,
    invert_y: bool = False,
) -> Tuple[float, float]:
    """
    Map a face-center pixel position to normalized [0,1] gaze coords.

    ``track_range`` is the fraction of the frame (half-width/half-height) that
    maps to full deflection. E.g. 0.6 means: if the face moves to 60% of the way
    from center to the edge, the eyes are already at full deflection. Smaller =
    more sensitive.

    Returns (nx, ny) where 0.5 is centered.
    """
    cx = face_cx / frame_w - 0.5   # -0.5 .. 0.5
    cy = face_cy / frame_h - 0.5

    # Convert to [-1, 1] over the track range.
    ox = (cx / (track_range / 2.0))
    oy = (cy / (track_range / 2.0))

    ox = apply_deadzone(max(-1.0, min(1.0, ox)), deadzone)
    oy = apply_deadzone(max(-1.0, min(1.0, oy)), deadzone)

    nx = 0.5 + ox * 0.5
    ny = 0.5 + oy * 0.5
    if invert_y:
        ny = 1.0 - ny
    return nx, ny


def pupil_offset_to_servo(
    offset_x: float,
    offset_y: float,
    gain: float = 1.0,
    deadzone: float = 0.05,
    invert_y: bool = False,
) -> Tuple[int, int]:
    """
    Map a pupil offset (in eye-ROI fractions, [-0.5, 0.5], 0 == centered) to
    firmware servo values. ``gain`` amplifies small offsets; ``deadzone`` kills
    jitter.
    """
    ox = apply_deadzone(max(-1.0, min(1.0, offset_x * 2.0 * gain)), deadzone)
    oy = apply_deadzone(max(-1.0, min(1.0, offset_y * 2.0 * gain)), deadzone)
    nx = 0.5 + ox * 0.5
    ny = 0.5 + oy * 0.5
    if invert_y:
        ny = 1.0 - ny
    return norm_to_servo(nx), norm_to_servo(ny)


# =============================================================================
# Serial sender  (real port via pyserial, or mock to stdout)
# =============================================================================

class SerialSender:
    """Writes ``"x,y\\n"`` to a serial port, or to stdout in mock mode."""

    def __init__(self, port: Optional[str], baud: int = 9600, mock: bool = False):
        self.mock = mock
        self.port = port
        self.baud = baud
        self._ser = None
        if not mock and port:
            self._open()

    def _open(self):
        try:
            import serial  # pyserial
        except ImportError as e:  # pragma: no cover
            raise SystemExit(
                "pyserial not installed. `pip install pyserial` or run with --mock."
            ) from e
        self._ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)  # Arduino resets on connect; let it boot.
        # Drain any boot banner.
        try:
            self._ser.read_all()
        except Exception:
            pass

    def write(self, x: int, y: int) -> None:
        text = f"{int(x)},{int(y)}\n"
        if self.mock or self._ser is None:
            sys.stdout.write(text)  # stdout wants str
            sys.stdout.flush()
        else:
            self._ser.write(text.encode("ascii"))  # pyserial wants bytes

    def close(self):
        if self._ser is not None:
            self._ser.close()


def list_ports() -> None:
    try:
        import serial.tools.list_ports
    except ImportError:
        raise SystemExit("pyserial not installed; cannot list ports.")
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device:18} {p.description} [{p.manufacturer}]")


# =============================================================================
# Trackers
# =============================================================================

@dataclass
class TrackResult:
    found: bool
    x: int = CENTER          # servo value 0..1023
    y: int = CENTER
    debug: dict = field(default_factory=dict)


class FaceTracker:
    """Haar-cascade face + eye detection. Returns servo gaze coords."""

    def __init__(self, mode: str = "face", deadzone: float = 0.04,
                 track_range: float = 0.6, invert_y: bool = False,
                 pupil_gain: float = 1.0):
        if cv2 is None:
            raise SystemExit("opencv-python not installed (pip install opencv-python).")
        self.mode = mode
        self.deadzone = deadzone
        self.track_range = track_range
        self.invert_y = invert_y
        self.pupil_gain = pupil_gain

        base = cv2.data.haarcascades
        self.face_cascade = cv2.CascadeClassifier(base + "haarcascade_frontalface_default.xml")
        self.eye_cascade = cv2.CascadeClassifier(base + "haarcascade_eye.xml")
        if self.face_cascade.empty() or self.eye_cascade.empty():
            raise SystemExit("Failed to load Haar cascades from cv2.data.haarcascades.")

    def _detect_pupil(self, eye_roi: "np.ndarray") -> Tuple[float, float]:
        """
        Rough pupil centroid inside an eye ROI (grayscale BGR already converted).
        Returns offset from ROI center as fractions in [-0.5, 0.5].
        """
        gray = cv2.cvtColor(eye_roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        # Threshold dark pixels (pupil + lashes). Pupil is the big central blob.
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        # Keep only the lower 2/3 (avoids eyebrow/upper-lash noise).
        h = th.shape[0]
        th[: h // 3, :] = 0
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0.0, 0.0
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < 8:
            return 0.0, 0.0
        m = cv2.moments(c)
        if m["m00"] == 0:
            return 0.0, 0.0
        cx = m["m10"] / m["m00"] / th.shape[1] - 0.5
        cy = m["m01"] / m["m00"] / th.shape[0] - 0.5
        return cx, cy

    def process(self, frame: "np.ndarray") -> TrackResult:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        if len(faces) == 0:
            return TrackResult(found=False, debug={"faces": 0})

        # Largest face.
        x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])

        if self.mode == "face":
            cx, cy = x + fw / 2.0, y + fh / 2.0
            nx, ny = face_to_norm(cx, cy, w, h, self.track_range,
                                  self.deadzone, self.invert_y)
            return TrackResult(found=True, x=norm_to_servo(nx), y=norm_to_servo(ny),
                               debug={"faces": len(faces), "mode": "face"})

        # pupil mode: need eyes
        roi_gray = gray[y:y + fh, x:x + fw]
        roi_color = frame[y:y + fh, x:x + fw]
        eyes = self.eye_cascade.detectMultiScale(roi_gray, 1.1, 3)
        if len(eyes) == 0:
            return TrackResult(found=False, debug={"faces": len(faces), "eyes": 0})
        # Average pupil offset across detected eyes.
        ox = oy = 0.0
        n = 0
        for (ex, ey, ew, eh) in eyes:
            eye_img = roi_color[ey:ey + eh, ex:ex + ew]
            px, py = self._detect_pupil(eye_img)
            ox += px
            oy += py
            n += 1
        ox /= n
        oy /= n
        sx, sy = pupil_offset_to_servo(ox, oy, self.pupil_gain,
                                       self.deadzone, self.invert_y)
        return TrackResult(found=True, x=sx, y=sy,
                           debug={"faces": len(faces), "eyes": len(eyes), "mode": "pupil"})


# =============================================================================
# EMA smoothing helper
# =============================================================================

class EMA:
    def __init__(self, alpha: float = 0.4):
        self.alpha = alpha
        self.x = float(CENTER)
        self.y = float(CENTER)
        self.seeded = False

    def update(self, x: int, y: int) -> Tuple[int, int]:
        if not self.seeded:
            self.x, self.y = float(x), float(y)
            self.seeded = True
        else:
            self.x += self.alpha * (x - self.x)
            self.y += self.alpha * (y - self.y)
        return int(round(self.x)), int(round(self.y))


# =============================================================================
# Main loop
# =============================================================================

def run(args) -> int:
    tracker = FaceTracker(
        mode=args.mode, deadzone=args.deadzone, track_range=args.track_range,
        invert_y=args.invert_y, pupil_gain=args.pupil_gain,
    )
    sender = SerialSender(args.port, args.baud, mock=args.mock)
    ema = EMA(args.smooth) if args.smooth > 0 else None

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera index {args.camera}.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print(f"[Eyemech] mode={args.mode} port={args.port or '(mock)'} "
          f"camera={args.camera} — press 'q' to quit", file=sys.stderr)

    hold_x, hold_y = CENTER, CENTER  # last good value when no detection
    no_detect_frames = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            frame = cv2.flip(frame, 1)  # mirror so it feels like a mirror

            res = tracker.process(frame)
            if res.found:
                no_detect_frames = 0
                hold_x, hold_y = res.x, res.y
            else:
                no_detect_frames += 1
                # Ease back toward center if we lose the face for a while.
                if no_detect_frames > args.lose_timeout:
                    hold_x += (CENTER - hold_x) * 0.05
                    hold_y += (CENTER - hold_y) * 0.05

            tx, ty = hold_x, hold_y
            if ema is not None:
                tx, ty = ema.update(tx, ty)

            sender.write(tx, ty)
            if args.verbose:
                print(f"  gaze=({tx},{ty}) {res.debug}", file=sys.stderr)

            if args.preview:
                cv2.imshow("Eyemech", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        sender.close()
        if args.preview:
            cv2.destroyAllWindows()
    return 0


# =============================================================================
# Self-test (no camera / no serial needed)
# =============================================================================

def run_selftest() -> int:
    print("Running Eyemech mapping self-test...", file=sys.stderr)
    failures = 0

    # norm_to_servo endpoints + center.
    assert norm_to_servo(0.0) == 0, "norm 0 -> 0"
    assert norm_to_servo(1.0) == 1023, "norm 1 -> 1023"
    assert norm_to_servo(0.5) == CENTER, f"norm .5 -> {CENTER}"
    assert 0 <= norm_to_servo(0.73) <= 1023

    # clamp never escapes range.
    for v in (-5.0, 0.0, 0.5, 1.0, 2.0):
        assert SERVO_MIN <= clamp_servo(v) <= SERVO_MAX

    # deadzone kills tiny offsets.
    assert apply_deadzone(0.02, 0.04) == 0.0
    assert apply_deadzone(-0.02, 0.04) == 0.0
    assert apply_deadzone(0.5, 0.04) != 0.0

    # face_to_norm: centered face -> 512,512
    nx, ny = face_to_norm(320, 240, 640, 480, track_range=0.6)
    assert abs(nx - 0.5) < 1e-6 and abs(ny - 0.5) < 1e-6, f"centered -> {nx},{ny}"

    # face far right -> x > 512
    nx2, _ = face_to_norm(600, 240, 640, 480, track_range=0.6)
    assert nx2 > 0.5, f"right face -> {nx2}"

    # invert_y flips vertical
    _, ny_a = face_to_norm(320, 100, 640, 480, track_range=0.6, invert_y=False)
    _, ny_b = face_to_norm(320, 100, 640, 480, track_range=0.6, invert_y=True)
    assert ny_a != ny_b and abs(ny_a + ny_b - 1.0) < 1e-6, f"invert_y {ny_a},{ny_b}"

    # pupil_offset_to_servo centered -> 512,512
    px, py = pupil_offset_to_servo(0.0, 0.0)
    assert px == CENTER and py == CENTER, f"pupil center -> {px},{py}"

    # pupil right offset -> x > 512
    px2, _ = pupil_offset_to_servo(0.2, 0.0)
    assert px2 > CENTER, f"pupil right -> {px2}"

    # EMA converges
    e = EMA(0.5)
    for _ in range(20):
        vx, vy = e.update(1023, 0)
    assert vx == 1023 and vy == 0, f"EMA converge -> {vx},{vy}"

    if failures:  # pragma: no cover
        print(f"FAILED {failures} checks.", file=sys.stderr)
        return 1
    print("OK — all mapping checks passed (0..1023 range respected).", file=sys.stderr)
    return 0


# =============================================================================
# CLI
# =============================================================================

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Eyemech OpenCV gaze controller (sends x,y over serial to the Arduino).")
    p.add_argument("--port", default=None,
                   help="Serial port, e.g. /dev/ttyUSB0 or COM3. Omit + --mock for stdout.")
    p.add_argument("--baud", type=int, default=9600, help="Baud rate (default 9600).")
    p.add_argument("--mock", action="store_true",
                   help="Print protocol lines to stdout instead of a serial port.")
    p.add_argument("--mode", choices=["face", "pupil"], default="face",
                   help="Tracking mode (default face-follow).")
    p.add_argument("--camera", type=int, default=0, help="Camera index (default 0).")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--track-range", type=float, default=0.6,
                   help="Face-follow sensitivity (lower = more sensitive).")
    p.add_argument("--pupil-gain", type=float, default=1.0,
                   help="Amplify pupil offsets in pupil mode.")
    p.add_argument("--deadzone", type=float, default=0.04,
                   help="Deadzone around center in normalized units.")
    p.add_argument("--invert-y", action="store_true",
                   help="Flip vertical axis to match your servo mounting.")
    p.add_argument("--smooth", type=float, default=0.0,
                   help="EMA alpha (0 = off). Firmware already smooths; keep small.")
    p.add_argument("--lose-timeout", type=int, default=15,
                   help="Frames without detection before easing eyes to center.")
    p.add_argument("--preview", action="store_true", help="Show webcam window.")
    p.add_argument("--verbose", action="store_true", help="Log each gaze value.")
    p.add_argument("--list-ports", action="store_true", help="List serial ports and exit.")
    p.add_argument("--selftest", action="store_true", help="Run mapping self-test and exit.")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if args.list_ports:
        list_ports()
        return 0
    if not args.mock and not args.port:
        # Be helpful: try to auto-detect a single port, else force mock.
        try:
            import serial.tools.list_ports
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if len(ports) == 1:
                args.port = ports[0]
                print(f"[Eyemech] auto-selected port {args.port}", file=sys.stderr)
            else:
                print("[Eyemech] no --port given and ports ambiguous; use --mock "
                      "or --list-ports.", file=sys.stderr)
                return 2
        except ImportError:
            print("[Eyemech] pyserial missing; use --mock.", file=sys.stderr)
            return 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
