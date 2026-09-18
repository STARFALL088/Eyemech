/*
 * Eyemech firmware — calibrated serial gaze + lid control
 *
 * Protocol (9600 baud):
 *   "<right_deg>,<up_deg>\n"                      — gaze; lids unchanged
 *   "<right_deg>,<up_deg>,<left_open>,<right_open>\n"
 *       left_open / right_open ∈ [0,1]  (1=open, 0=closed) — person's L/R
 *       drives mechanism UL/LL and UR/LR independently
 *   "HOME\n"                                      — park at calibrated zero
 *
 * Calibrated ranges (from experimental sweep maps):
 *   D3  X:     0  .. 80     (mid / look-straight = 40)
 *   D5  Y:    60  .. 120    (mid / look-straight = 90)
 *   D6  UL:  blink open=130 / closed=180  (polarity flipped vs first calib pass)
 *   D9  LL:  blink open=50  / closed=0
 *   D10 UR:  blink open=50  / closed=0
 *   D11 LR:  blink open=130 / closed=180
 *
 * Gaze: host +right / +up map directly to servo (no axis negate).
 * Lids follow host (no auto-blink); mechanical HOME = experimental v=0.
 */

#include <Servo.h>

// ---------------------------------------------------------------------------
// Tunables — host-degree limits from mechanism calibrator (pad / full iris look)
// Iris θ→deg scaling lives in pupil_control.py (eye_calibrate extremes).
// ---------------------------------------------------------------------------
const float MAX_TURN_X = 14.0;   // ±right  (L/R extremes)
const float MAX_TURN_Y = 4.5;    // ±up     (U/D extremes)

const unsigned long BAUD = 9600;

// ---------------------------------------------------------------------------
// Pins (match wired hardware / experimental sketch)
// ---------------------------------------------------------------------------
const int PIN_SERVO_X  = 3;
const int PIN_SERVO_Y  = 5;
const int PIN_BLINK_UL = 6;
const int PIN_BLINK_LL = 9;
const int PIN_BLINK_UR = 10;
const int PIN_BLINK_LR = 11;

// Eyeball calibrated endpoints (v=0 .. v=1) and look-straight midpoints
const int X_MIN = 0,   X_MAX = 80,  X_MID = 40;
const int Y_MIN = 60,  Y_MAX = 120, Y_MID = 90;

// Eyelid open / closed (blink polarity flipped)
const int UL_OPEN = 130, UL_CLOSED = 180;
const int LL_OPEN = 50,  LL_CLOSED = 0;
const int UR_OPEN = 50,  UR_CLOSED = 0;
const int LR_OPEN = 130, LR_CLOSED = 180;

// Full mechanical park = experimental v=0 (not the same as blink "open")
const int HOME_X  = X_MIN;  // 0
const int HOME_Y  = Y_MIN;  // 60
const int HOME_UL = 180;
const int HOME_LL = 0;
const int HOME_UR = 0;
const int HOME_LR = 180;

Servo servoX, servoY, blinkUL, blinkLL, blinkUR, blinkLR;

float cmdRightDeg = 0.0;
float cmdUpDeg    = 0.0;
float cmdLeftOpen  = 1.0;
float cmdRightOpen = 1.0;

String serialBuffer;

bool parkedHome = false;   // after HOME, hold until next gaze cmd

// ---------------------------------------------------------------------------
float safe_turn_axis(float angle, float limit) {
  return max(-limit, min(limit, angle));
}

float clampf(float v, float lo, float hi) {
  return max(lo, min(hi, v));
}

// Map host degrees → calibrated servo angle (0 deg → MID, ±limit → ends).
int mapGazeToServo(float deg, int servoMin, int servoMax, int servoMid, float limit) {
  float a = safe_turn_axis(deg, limit);
  if (limit <= 0.0f) {
    return servoMid;
  }
  if (a >= 0.0f) {
    return servoMid + (int)round((a / limit) * (servoMax - servoMid));
  }
  return servoMid + (int)round((a / limit) * (servoMid - servoMin));
}

// open01: 1 = fully open endpoint, 0 = fully closed endpoint.
int mapLidOpen(float open01, int openPos, int closedPos) {
  float o = clampf(open01, 0.0f, 1.0f);
  return (int)round(openPos * o + closedPos * (1.0f - o));
}

void applyGaze() {
  parkedHome = false;
  servoX.write(mapGazeToServo(cmdRightDeg, X_MIN, X_MAX, X_MID, MAX_TURN_X));
  servoY.write(mapGazeToServo(cmdUpDeg,    Y_MIN, Y_MAX, Y_MID, MAX_TURN_Y));
}

void applyLids() {
  parkedHome = false;
  // Person's left → mechanism UL/LL; person's right → UR/LR.
  blinkUL.write(mapLidOpen(cmdLeftOpen,  UL_OPEN, UL_CLOSED));
  blinkLL.write(mapLidOpen(cmdLeftOpen,  LL_OPEN, LL_CLOSED));
  blinkUR.write(mapLidOpen(cmdRightOpen, UR_OPEN, UR_CLOSED));
  blinkLR.write(mapLidOpen(cmdRightOpen, LR_OPEN, LR_CLOSED));
}

void goHome() {
  servoX.write(HOME_X);
  servoY.write(HOME_Y);
  blinkUL.write(HOME_UL);
  blinkLL.write(HOME_LL);
  blinkUR.write(HOME_UR);
  blinkLR.write(HOME_LR);
  parkedHome = true;
  cmdRightDeg = 0.0;
  cmdUpDeg    = 0.0;
  cmdLeftOpen  = 1.0;
  cmdRightOpen = 1.0;
  Serial.println("HOME");
}

void handleLine(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) {
    return;
  }

  String up = s;
  up.toUpperCase();
  if (up == "HOME") {
    goHome();
    return;
  }

  int c1 = s.indexOf(',');
  if (c1 < 0) {
    return;
  }
  int c2 = s.indexOf(',', c1 + 1);
  cmdRightDeg = s.substring(0, c1).toFloat();
  if (c2 < 0) {
    cmdUpDeg = s.substring(c1 + 1).toFloat();
    applyGaze();
    return;
  }
  int c3 = s.indexOf(',', c2 + 1);
  cmdUpDeg = s.substring(c1 + 1, c2).toFloat();
  if (c3 < 0) {
    applyGaze();
    return;
  }
  cmdLeftOpen  = clampf(s.substring(c2 + 1, c3).toFloat(), 0.0f, 1.0f);
  cmdRightOpen = clampf(s.substring(c3 + 1).toFloat(), 0.0f, 1.0f);
  applyGaze();
  applyLids();
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleLine(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) {
        serialBuffer = "";
      }
    }
  }
}

void setup() {
  Serial.begin(BAUD);

  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);
  blinkUL.attach(PIN_BLINK_UL);
  blinkLL.attach(PIN_BLINK_LL);
  blinkUR.attach(PIN_BLINK_UR);
  blinkLR.attach(PIN_BLINK_LR);

  // Assume mechanism is already at HOME — do not write positions on boot.
  parkedHome = true;
  Serial.println("Eyemech firmware ready (gaze+lids)");
}

void loop() {
  readSerial();
}
