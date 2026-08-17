//  Nilheim Mechatronics Eye Mechanism - OpenCV Gaze Control Version
//  Reads X/Y gaze data over Serial from a Python/OpenCV script instead of a joystick
//  Expects lines like: "512,300\n"  (xval,yval both 0-1023)
//
//  Trim potentiometer pin: A2 (still used for manual eyelid calibration)
//  Button pin: 2 (still used for manual "eyes wide open" override)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  140
#define SERVOMAX  520

int xval = 512;   // default to center until first serial data arrives
int yval = 512;

int lexpulse;
int rexpulse;

int leypulse;
int reypulse;

int uplidpulse;
int lolidpulse;
int altuplidpulse;
int altlolidpulse;

int trimval;

int switchval = 0;

// --- Serial smoothing ---
const int SMOOTH_SAMPLES = 5;
int xHistory[SMOOTH_SAMPLES];
int yHistory[SMOOTH_SAMPLES];
int historyIndex = 0;

// --- Serial parsing buffer ---
String serialBuffer = "";

void setup() {
  Serial.begin(9600);
  Serial.println("Eye Mechanism - OpenCV Serial Control");
  pinMode(2, INPUT);

  pwm.begin();
  pwm.setPWMFreq(60);

  for (int i = 0; i < SMOOTH_SAMPLES; i++) {
    xHistory[i] = 512;
    yHistory[i] = 512;
  }

  delay(10);
}

void readSerialGaze() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int commaIndex = serialBuffer.indexOf(',');
      if (commaIndex > 0) {
        int newX = serialBuffer.substring(0, commaIndex).toInt();
        int newY = serialBuffer.substring(commaIndex + 1).toInt();

        newX = constrain(newX, 0, 1023);
        newY = constrain(newY, 0, 1023);

        // rolling average for smoothing
        xHistory[historyIndex] = newX;
        yHistory[historyIndex] = newY;
        historyIndex = (historyIndex + 1) % SMOOTH_SAMPLES;

        long xSum = 0, ySum = 0;
        for (int i = 0; i < SMOOTH_SAMPLES; i++) {
          xSum += xHistory[i];
          ySum += yHistory[i];
        }
        xval = xSum / SMOOTH_SAMPLES;
        yval = ySum / SMOOTH_SAMPLES;
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

void loop() {

  readSerialGaze();   // updates xval, yval from Python instead of analogRead

  lexpulse = map(xval, 0, 1023, 220, 440);
  rexpulse = lexpulse;

  switchval = digitalRead(2);

  leypulse = map(yval, 0, 1023, 250, 500);
  reypulse = map(yval, 0, 1023, 400, 280);

  trimval = analogRead(A2);
  trimval = map(trimval, 320, 580, -40, 40);

  uplidpulse = map(yval, 0, 1023, 400, 280);
  uplidpulse -= (trimval - 40);
  uplidpulse = constrain(uplidpulse, 280, 400);
  altuplidpulse = 680 - uplidpulse;

  lolidpulse = map(yval, 0, 1023, 410, 280);
  lolidpulse += (trimval / 2);
  lolidpulse = constrain(lolidpulse, 280, 400);
  altlolidpulse = 680 - lolidpulse;

  pwm.setPWM(0, 0, lexpulse);
  pwm.setPWM(1, 0, leypulse);

  if (switchval == HIGH) {
    pwm.setPWM(2, 0, 400);
    pwm.setPWM(3, 0, 240);
    pwm.setPWM(4, 0, 240);
    pwm.setPWM(5, 0, 400);
  }
  else if (switchval == LOW) {
    pwm.setPWM(2, 0, uplidpulse);
    pwm.setPWM(3, 0, lolidpulse);
    pwm.setPWM(4, 0, altuplidpulse);
    pwm.setPWM(5, 0, altlolidpulse);
  }

  delay(5);
}
