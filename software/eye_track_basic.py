#!/usr/bin/env python3
"""
Eyemech — BASIC eye tracking demo
=================================

The smallest useful version of the computer side: find a face, find each eye,
locate the pupil (the dark blob) inside the eye, and report where you're
looking. No servo/serial complexity — just the vision.

Two ways to run:

  # Live webcam (your machine, needs opencv-python + a camera):
  python3 eye_track_basic.py
  python3 eye_track_basic.py --port /dev/ttyUSB0   # also drive the Arduino

  # Headless test on a still image (no camera needed):
  python3 eye_track_basic.py --image eye.png

In live mode press 'q' to quit. The script prints, for each eye, the pupil
offset from the eye center as a fraction in [-0.5, 0.5] (0 = centered), and
optionally the Arduino-style "x,y" line.

If --port is given it writes "x,y\\n" (0..1023, 512=center) to the Arduino,
matching Basic movements for openCV.ino.
"""

from __future__ import annotations
import argparse
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


# ---------------------------------------------------------------------------
# Pupil detection (the core of "track eye")
# ---------------------------------------------------------------------------
def find_pupil(eye_roi: np.ndarray):
    """
    Given a BGR eye crop, return (offset_x, offset_y, radius) where offsets are
    fractions of the ROI in [-0.5, 0.5] (0 = eye center). Returns None if no
    pupil found.

    Method: keep only the darkest pixels (the pupil, not the mid-tone iris) using
    a relative threshold below the local mean, then take the roundest sizable
    blob. Top third is masked to avoid eyebrow / upper-lash noise.
    """
    if eye_roi.size == 0:
        return None
    gray = cv2.cvtColor(eye_roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    h, w = gray.shape
    # The pupil is the DARKEST region of the eye (near-black). Threshold just
    # above the global minimum so only the pupil survives (the iris, even when
    # dark, stays above this and won't merge in and bias the centroid).
    thr = int(gray.min()) + 30
    _, th = cv2.threshold(gray, thr, 255, cv2.THRESH_BINARY_INV)
    th[: h // 3, :] = 0  # drop the top third (brow/lashes)
    kernel = np.ones((2, 2), np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)

    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    # Prefer the roundest blob that is at least 4px — the pupil is compact & circular.
    best = None
    best_score = -1.0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 4:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        if radius < 1.5 or radius > min(w, h) / 2.0:
            continue
        # Reject blobs that are tiny relative to the eye (false specks) or
        # sitting far outside the eye center (stray lashes/reflections).
        off = abs(cx / w - 0.5) + abs(cy / h - 0.5)
        if radius < min(w, h) * 0.06 or off > 0.45:
            continue
        # Circularity: 4*pi*area / perimeter^2  (1.0 = perfect circle)
        peri = cv2.arcLength(c, True)
        circ = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
        score = circ
        if score > best_score:
            best_score = score
            best = (cx / w - 0.5, cy / h - 0.5, float(radius))
    return best


# ---------------------------------------------------------------------------
# Face + eye detection
# ---------------------------------------------------------------------------
def load_cascades():
    if cv2 is None:
        raise SystemExit("opencv-python not installed (pip install opencv-python).")
    base = cv2.data.haarcascades
    face = cv2.CascadeClassifier(base + "haarcascade_frontalface_default.xml")
    eye = cv2.CascadeClassifier(base + "haarcascade_eye.xml")
    if face.empty() or eye.empty():
        raise SystemExit("Failed to load Haar cascades.")
    return face, eye


def process_frame(frame: np.ndarray, face_cascade, eye_cascade, draw: bool = True):
    """
    Detect face + eyes, find pupils. Returns list of dicts:
      {ex, ey, ew, eh, ox, oy, r}  (ox/oy = pupil offset fractions)
    and optionally an annotated copy of the frame.
    """
    out = frame.copy() if draw else frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    results = []
    for (x, y, fw, fh) in faces:
        if draw:
            cv2.rectangle(out, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
        roi_g = gray[y:y + fh, x:x + fw]
        roi_c = frame[y:y + fh, x:x + fw]
        eyes = eye_cascade.detectMultiScale(roi_g, 1.1, 3)
        for (ex, ey, ew, eh) in eyes:
            eye_img = roi_c[ey:ey + eh, ex:ex + ew]
            pup = find_pupil(eye_img)
            ox = oy = 0.0
            r = 0.0
            if pup is not None:
                ox, oy, r = pup
            results.append({"ex": ex + x, "ey": ey + y, "ew": ew, "eh": eh,
                            "ox": ox, "oy": oy, "r": r})
            if draw:
                cv2.rectangle(out, (ex + x, ey + y), (ex + x + ew, ey + y + eh),
                              (255, 0, 0), 1)
                # mark pupil center relative to eye ROI
                px = int(ex + x + (ox + 0.5) * ew)
                py = int(ey + y + (oy + 0.5) * eh)
                cv2.circle(out, (px, py), max(2, int(r)), (0, 0, 255), 2)
    return results, (out if draw else frame)


# ---------------------------------------------------------------------------
# Arduino output
# ---------------------------------------------------------------------------
def offsets_to_servo(ox: float, oy: float, invert_y: bool = False):
    """Average pupil offset (fractions) -> 0..1023, 512 = center."""
    ox = max(-0.5, min(0.5, ox))
    oy = max(-0.5, min(0.5, oy))
    nx = 0.5 + ox * 2.0 * 0.5  # map [-0.5,0.5] -> [0,1] scaled to half-range
    ny = 0.5 + oy * 2.0 * 0.5
    if invert_y:
        ny = 1.0 - ny
    return int(round(nx * 1023)), int(round(ny * 1023))


def main():
    ap = argparse.ArgumentParser(description="Eyemech basic eye tracker.")
    ap.add_argument("--image", help="Run on a still image instead of webcam.")
    ap.add_argument("--port", help="If set, also write x,y to this serial port.")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--invert-y", action="store_true")
    ap.add_argument("--out", help="Save annotated image here (headless).")
    ap.add_argument("--mock", action="store_true", help="Print x,y to stdout.")
    args = ap.parse_args()

    sender = None
    if args.port or args.mock:
        try:
            import serial
            if args.port:
                sender = serial.Serial(args.port, 9600, timeout=1)
                time.sleep(2)
        except ImportError:
            print("pyserial not installed; --mock only.", file=sys.stderr)

    face_c, eye_c = load_cascades()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f"Cannot read image: {args.image}")
        img = cv2.flip(img, 1)
        results, annotated = process_frame(img, face_c, eye_c, draw=True)
        for i, r in enumerate(results):
            print(f"eye{i}: offset=({r['ox']:+.3f},{r['oy']:+.3f}) "
                  f"r={r['r']:.1f} box=({r['ex']},{r['ey']},{r['ew']}x{r['eh']})")
        if args.out:
            cv2.imwrite(args.out, annotated)
            print(f"saved annotated -> {args.out}")
        return

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera {args.camera}.")
    print("Live mode — press 'q' to quit.", file=sys.stderr)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            frame = cv2.flip(frame, 1)
            results, annotated = process_frame(frame, face_c, eye_c, draw=True)
            # Average the eyes for a single gaze point.
            if results:
                ox = sum(r["ox"] for r in results) / len(results)
                oy = sum(r["oy"] for r in results) / len(results)
                x, y = offsets_to_servo(ox, oy, args.invert_y)
                line = f"{x},{y}"
                if sender:
                    sender.write((line + "\n").encode())
                if args.mock:
                    sys.stdout.write(line + "\n")
                    sys.stdout.flush()
                cv2.putText(annotated, line, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Eyemech basic", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if sender:
            sender.close()


if __name__ == "__main__":
    main()
