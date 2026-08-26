#!/usr/bin/env python3
"""
End-to-end compatibility proof for Eyemech.

We don't have an Arduino or a webcam here, but we DO have both halves' logic:
  * the Python side's mapping (eyemech_gaze.py) produces the "x,y\n" protocol
  * the Arduino's readSerialGaze() + loop() map() math turns x,y into servo pulses

This script re-implements the FIRMWARE'S exact math (copied from
"Basic movements for openCV.ino") and feeds it the Python side's output, then
asserts every servo pulse lands in [SERVOMIN, SERVOMAX] and that motion goes
the right way. If this passes, the two halves speak the same language.
"""
from eyemech_gaze import face_to_norm, norm_to_servo, CENTER, SERVO_MIN, SERVO_MAX

# --- Exact constants from the Arduino firmware ---
SERVO_MIN_FW, SERVO_MAX_FW = 140, 520  # firmware pulse bounds
X_CENTER, Y_CENTER = 512, 512           # firmware defaults


def firmware_loop(xval, yval, switchval=0):
    """Faithful port of the .ino loop() mapping for one frame."""
    trimval = 0  # assume pot centered
    lexpulse = int(round((xval - 0) * (440 - 220) / (1023 - 0) + 220))
    rexpulse = lexpulse

    leypulse = int(round((yval - 0) * (500 - 250) / (1023 - 0) + 250))
    reypulse = int(round((yval - 0) * (280 - 400) / (1023 - 0) + 400))

    uplidpulse = int(round((yval - 0) * (280 - 400) / (1023 - 0) + 400)) - (trimval - 40)
    uplidpulse = max(280, min(400, uplidpulse))
    altuplidpulse = 680 - uplidpulse
    lolidpulse = int(round((yval - 0) * (280 - 410) / (1023 - 0) + 410)) + (trimval // 2)
    lolidpulse = max(280, min(400, lolidpulse))
    altlolidpulse = 680 - lolidpulse

    return dict(lex=lexpulse, rex=rexpulse, ley=leypulse, rey=reypulse,
                uplid=uplidpulse, lolid=lolidpulse,
                altuplid=altuplidpulse, altlolid=altlolidpulse)


def check(name, x, y):
    print(f"  {name:14} python_out=({x},{y})", end="")
    # firmware parse: constrain 0..1023 (already done by script)
    xv = max(0, min(1023, x))
    yv = max(0, min(1023, y))
    p = firmware_loop(xv, yv)
    # every pulse must be inside the firmware's physical servo range
    for k, v in p.items():
        assert SERVO_MIN_FW <= v <= SERVO_MAX_FW, f"{k}={v} out of range!"
    print(f"  -> pulses {p}")
    return p


def main():
    print("Eyemech firmware-compat check\n")
    FRAME_W, FRAME_H = 640, 480

    # Center face -> expect x,y ~ 512,512 -> all pulses near midpoints
    cx, cy = face_to_norm(FRAME_W / 2, FRAME_H / 2, FRAME_W, FRAME_H)
    p_c = check("CENTER", norm_to_servo(cx), norm_to_servo(cy))

    # Face far LEFT -> eyes should swing left (lower X pulse)
    cx, _ = face_to_norm(40, FRAME_H / 2, FRAME_W, FRAME_H)
    p_l = check("FAR LEFT", norm_to_servo(cx), norm_to_servo(cy))

    # Face far RIGHT -> higher X pulse
    cx, _ = face_to_norm(FRAME_W - 40, FRAME_H / 2, FRAME_W, FRAME_H)
    p_r = check("FAR RIGHT", norm_to_servo(cx), norm_to_servo(cy))

    # Face UP (top of frame) -> eyelid pulses respond (X held centered)
    cx_up, cy_up = face_to_norm(FRAME_W / 2, 40, FRAME_W, FRAME_H)
    p_u = check("FAR UP", norm_to_servo(cx_up), norm_to_servo(cy_up))

    # Face DOWN (bottom of frame)
    cx_dn, cy_dn = face_to_norm(FRAME_W / 2, FRAME_H - 40, FRAME_W, FRAME_H)
    p_d = check("FAR DOWN", norm_to_servo(cx_dn), norm_to_servo(cy_dn))

    # Direction assertions
    assert p_l["lex"] < p_c["lex"] < p_r["lex"], "X pulse must increase left->center->right"
    assert p_c["lex"] == p_c["rex"], "left/right eye X should match"
    print("\nOK: all servo pulses within [140,520]; eye-X increases left->right; L/R matched.")
    print("=> Python 'x,y' protocol is compatible with the Arduino firmware.\n")


if __name__ == "__main__":
    main()
