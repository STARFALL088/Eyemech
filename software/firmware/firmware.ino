/*
 * Eyemech firmware — direct serial gaze control
 *
 * 6 servos on sequential digital pins:
 *   X / Y eyeball  +  4 eyelid blink servos
 *
 * Protocol (9600 baud), one line per command from direct_control.py:
 *   "<right_deg>,<up_deg>\n"
 * Example:  "7.50,-3.20\n"
 *
 * Center look = 0,0.  safe_turn() clamps each axis to ±MAX_TURN_ANGLE
 * before writing the X/Y servos. Blink runs on its own millis() timer.
 */

#include <Servo.h>

// ---------------------------------------------------------------------------
// Tunables (adjust later to match hardware)
// ---------------------------------------------------------------------------
const float MAX_TURN_ANGLE = 15.0;   // degrees — hard safety cap for X/Y

const int SERVO_CENTER = 90;         // neutral eyeball position
const int LID_OPEN     = 90;         // eyelid open position
const int LID_CLOSED   = 50;         // eyelid closed position (tune per horn)

const unsigned long BLINK_INTERVAL_MS = 3000;  // time between blinks
const unsigned long BLINK_CLOSED_MS   = 150;   // how long lids stay shut

const unsigned long BAUD = 9600;

// ---------------------------------------------------------------------------
// Named servo pins (sequential digital pins — rewire freely later)
// ---------------------------------------------------------------------------
const int PIN_SERVO_X       = 2;  // eyeball horizontal (rightward +)
const int PIN_SERVO_Y       = 3;  // eyeball vertical   (upward +)
const int PIN_BLINK_UL      = 4;  // upper-left  eyelid
const int PIN_BLINK_LL      = 5;  // lower-left  eyelid
const int PIN_BLINK_UR      = 6;  // upper-right eyelid
const int PIN_BLINK_LR      = 7;  // lower-right eyelid

Servo servoX;
Servo servoY;
Servo blinkUL;
Servo blinkLL;
Servo blinkUR;
Servo blinkLR;

// Last commanded gaze (degrees, pre-clamp stored for debug; applied via safe_turn)
float cmdRightDeg = 0.0;
float cmdUpDeg    = 0.0;

String serialBuffer;

unsigned long lastBlinkAt   = 0;
unsigned long blinkCloseAt  = 0;
bool          lidsClosed    = false;

// ---------------------------------------------------------------------------
// Safety: never command more than ±MAX_TURN_ANGLE from center
// ---------------------------------------------------------------------------
float safe_turn(float angle) {
  // Clamp into [-MAX_TURN_ANGLE, +MAX_TURN_ANGLE] with max/min.
  return max(-MAX_TURN_ANGLE, min(MAX_TURN_ANGLE, angle));
}

void applyGaze() {
  float right = safe_turn(cmdRightDeg);
  float up    = safe_turn(cmdUpDeg);

  // +right / +up from pad → offset from SERVO_CENTER (tune signs later)
  servoX.write(SERVO_CENTER + (int)round(right));
  servoY.write(SERVO_CENTER + (int)round(up));
}

void setLidsOpen() {
  blinkUL.write(LID_OPEN);
  blinkLL.write(LID_OPEN);
  blinkUR.write(LID_OPEN);
  blinkLR.write(LID_OPEN);
  lidsClosed = false;
}

void setLidsClosed() {
  blinkUL.write(LID_CLOSED);
  blinkLL.write(LID_CLOSED);
  blinkUR.write(LID_CLOSED);
  blinkLR.write(LID_CLOSED);
  lidsClosed = true;
}

void updateBlink() {
  unsigned long now = millis();

  if (!lidsClosed && (now - lastBlinkAt >= BLINK_INTERVAL_MS)) {
    setLidsClosed();
    blinkCloseAt = now;
    lastBlinkAt  = now;
  } else if (lidsClosed && (now - blinkCloseAt >= BLINK_CLOSED_MS)) {
    setLidsOpen();
  }
}

// Parse one complete line: "right,up"
void handleLine(const String& line) {
  int comma = line.indexOf(',');
  if (comma < 0) {
    return;
  }
  float right = line.substring(0, comma).toFloat();
  float up    = line.substring(comma + 1).toFloat();
  cmdRightDeg = right;
  cmdUpDeg    = up;
  applyGaze();
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
      // Guard against garbage flooding the buffer.
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

  cmdRightDeg = 0.0;
  cmdUpDeg    = 0.0;
  applyGaze();
  setLidsOpen();

  lastBlinkAt = millis();
  Serial.println("Eyemech firmware ready");
}

void loop() {
  readSerial();
  updateBlink();
}
