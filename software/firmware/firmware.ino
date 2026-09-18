/*
 * Eyemech firmware — calibrated serial gaze control
 *
 * Protocol (9600 baud):
 *   "<right_deg>,<up_deg>\n"  — look command (0,0 = straight ahead / mid-range)
 *   "HOME\n"                  — park at calibrated zero (v=0 pose), then hold
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
 * Mechanical park HOME still uses experimental v=0: (0, 60, 180, 0, 0, 180).
 */

#include <Servo.h>

// ---------------------------------------------------------------------------
// Tunables — host-degree limits from mechanism calibrator (pad / full iris look)
// Iris θ→deg scaling lives in pupil_control.py (eye_calibrate extremes).
// ---------------------------------------------------------------------------
const float MAX_TURN_X = 14.0;   // ±right  (L/R extremes)
const float MAX_TURN_Y = 4.5;    // ±up     (U/D extremes)

const unsigned long BLINK_INTERVAL_MS = 3000;
const unsigned long BLINK_CLOSED_MS   = 150;
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

String serialBuffer;

unsigned long lastBlinkAt  = 0;
unsigned long blinkCloseAt = 0;
bool lidsClosed = false;
bool parkedHome = false;   // after HOME, ignore blink until next gaze cmd

// ---------------------------------------------------------------------------
float safe_turn_axis(float angle, float limit) {
  // Symmetric clamp to ±limit for one axis.
  return max(-limit, min(limit, angle));
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

void applyGaze() {
  parkedHome = false;
  servoX.write(mapGazeToServo(cmdRightDeg, X_MIN, X_MAX, X_MID, MAX_TURN_X));
  servoY.write(mapGazeToServo(cmdUpDeg,    Y_MIN, Y_MAX, Y_MID, MAX_TURN_Y));
}

void setLidsOpen() {
  blinkUL.write(UL_OPEN);
  blinkLL.write(LL_OPEN);
  blinkUR.write(UR_OPEN);
  blinkLR.write(LR_OPEN);
  lidsClosed = false;
}

void setLidsClosed() {
  blinkUL.write(UL_CLOSED);
  blinkLL.write(LL_CLOSED);
  blinkUR.write(UR_CLOSED);
  blinkLR.write(LR_CLOSED);
  lidsClosed = true;
}

void goHome() {
  servoX.write(HOME_X);
  servoY.write(HOME_Y);
  blinkUL.write(HOME_UL);
  blinkLL.write(HOME_LL);
  blinkUR.write(HOME_UR);
  blinkLR.write(HOME_LR);
  lidsClosed = false;
  parkedHome = true;
  cmdRightDeg = 0.0;
  cmdUpDeg    = 0.0;
  Serial.println("HOME");
}

void updateBlink() {
  if (parkedHome) {
    return;  // stay at home lids until next gaze command
  }
  unsigned long now = millis();
  if (!lidsClosed && (now - lastBlinkAt >= BLINK_INTERVAL_MS)) {
    setLidsClosed();
    blinkCloseAt = now;
    lastBlinkAt  = now;
  } else if (lidsClosed && (now - blinkCloseAt >= BLINK_CLOSED_MS)) {
    setLidsOpen();
  }
}

void handleLine(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) {
    return;
  }

  // Case-insensitive HOME
  String up = s;
  up.toUpperCase();
  if (up == "HOME") {
    goHome();
    return;
  }

  int comma = s.indexOf(',');
  if (comma < 0) {
    return;
  }
  cmdRightDeg = s.substring(0, comma).toFloat();
  cmdUpDeg    = s.substring(comma + 1).toFloat();
  applyGaze();
  // If lids were left closed mid-blink when a cmd arrives, leave blink SM as-is.
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
  lastBlinkAt = millis();
  Serial.println("Eyemech firmware ready (calibrated)");
}

void loop() {
  readSerial();
  updateBlink();
}
