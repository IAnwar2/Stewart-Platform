#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ---- Servo calibration (tune these if needed) ----
#define SERVOMIN     90     // ~0°
#define SERVOMAX     600     // ~180°
#define SERVO_FREQ   50      // 50 Hz hobby servos

// ---- App limits/neutral (same semantics as your first sketch) ----
const int MIN_ANGLE = -10;
const int MAX_ANGLE = 30;
const int NEUTRAL_ANGLE = 15;

// ---- Which PCA9685 channels have servos plugged in ----
const uint8_t SERVO_CHANNELS[] = {1, 2, 3};
const uint8_t NUM_SERVOS = sizeof(SERVO_CHANNELS) / sizeof(SERVO_CHANNELS[0]);

// ---- State per servo ----
int  targetAngle[NUM_SERVOS];
bool pendingWrite[NUM_SERVOS];

// Map 0–180° to PCA9685 ticks
int angleToPulse(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Write an angle to a *channel* (not index)
void writeServoAngle(uint8_t channel, int angle) {
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

// Find index in SERVO_CHANNELS for a given PCA9685 channel; returns -1 if not found
int indexForChannel(int ch) {
  for (uint8_t i = 0; i < NUM_SERVOS; ++i)
    if (SERVO_CHANNELS[i] == ch) return i;
  return -1;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize all servos to neutral
  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    targetAngle[i]  = NEUTRAL_ANGLE;
    pendingWrite[i] = true;
    writeServoAngle(SERVO_CHANNELS[i], NEUTRAL_ANGLE);
    pendingWrite[i] = false;
  }

  Serial.println("PCA9685 multi-servo controller ready.");
  Serial.println("Commands:");
  Serial.println("  <ch> <angle>   e.g. '2 15' (angle 0..30)");
  Serial.println("  all <angle>    e.g. 'all 10'");
  Serial.println("  neutral        (set all to 25)");
  Serial.println("  off <ch>       (disable pulses on channel)");
}

void loop() {
  // ---- Parse one full line ----
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) 
    {
      int firstSep = -1;
      for (uint16_t i = 0; i < line.length(); ++i) {
        char c = line[i];
        if (c == ' ') { firstSep = i; break; }
      }

      if (firstSep > 0) {
        String chStr = line.substring(0, firstSep); chStr.trim();
        String rest  = line.substring(firstSep + 1); rest.trim();

        int ch = chStr.toInt();
        // get angle = first integer in rest
        int angle = rest.toInt();

        // int idx = indexForChannel(ch);

        writeServoAngle(ch, angle);
      } 
    }
  }

  delay(10);
}