#include <Pixy2I2C.h>
#include <Servo.h>
#include "Ultrasonic.h"

#define MOTOR_SPEED 90
#define servo_center 90
#define target_distance 10.0
#define Kp 3.0
#define RECOVERY_DELAY 1000  // ms

Pixy2I2C pixy;
Ultrasonic depan(40);
Ultrasonic kanan(42);
Ultrasonic kiri(38);

Servo steering;
const int servoPin = 7;

// Motor control pins
const int in1Pin = 10;
const int in2Pin = 8;
const int in3Pin = 9;
const int in4Pin = 11;
const int enA_kanan1 = 19;
const int enB_kanan2 = 18;
const int enA_kiri1  = 3;
const int enB_kiri2  = 2;

// Mode gerakan
enum Mode { NORMAL, AVOID_RIGHT, AVOID_LEFT, AVOID_FRONT };
Mode currentMode = NORMAL;

unsigned long actionStartTime = 0;
unsigned long lastSig2Seen = 0, lastSig1Seen = 0;
unsigned long leftAvoidStartTime = 0, leftAvoidDuration = 0, recoveryStartTime = 0;
bool avoidingRight = false, avoidingLeft = false;
bool recoveryFromLeft = false;

// Sensor readings
long RangeDepan, RangeKanan, RangeKiri;

// Signal detection variables (global scope)
bool detect_sig2 = false, detect_sig1 = false;
int x_sig2 = -1, x_sig1 = -1;

void setup() {
  Serial.begin(115200);
  pixy.init();
  pixy.setLamp(1, 0); // Only upper LED ON
  steering.attach(servoPin);
  steering.write(servo_center);

  pinMode(in1Pin, OUTPUT); pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT); pinMode(in4Pin, OUTPUT);
  pinMode(enA_kanan1, OUTPUT); pinMode(enB_kanan2, OUTPUT);
  pinMode(enA_kiri1, OUTPUT);  pinMode(enB_kiri2, OUTPUT);

  analogWrite(enA_kanan1, MOTOR_SPEED);
  analogWrite(enB_kanan2, MOTOR_SPEED);
  analogWrite(enA_kiri1, MOTOR_SPEED);
  analogWrite(enB_kiri2, MOTOR_SPEED);
}

void loop() {
  updateSensors();
  detectSignals();
  unsigned long now = millis();

  if (currentMode == AVOID_FRONT) {
    handleFrontAvoid(now);
    return;
  }

  if (RangeDepan > 0 && RangeDepan < 5 && currentMode != AVOID_FRONT) {
    currentMode = AVOID_FRONT;
    actionStartTime = now;
    return;
  }

  if (recoveryFromLeft) {
    if (now - recoveryStartTime < leftAvoidDuration) {
      steering.write(120);
      forward(60);
      Serial.println("Recovery - belok kanan");
    } else {
      recoveryFromLeft = false;
      Serial.println("Recovery selesai - mode normal");
    }
    return;
  }

  updateAvoidLogic(now);

  if (currentMode == AVOID_RIGHT && detect_sig2) {
    proportionalAvoid(x_sig2 - 10, true);
  } else if (currentMode == AVOID_LEFT && detect_sig1) {
    proportionalAvoid(306 - x_sig1, false);
  } else {
    float error = target_distance - RangeKanan;
    float control = Kp * error;
    int steer_angle = constrain(servo_center + control, 60, 120);
    steering.write(steer_angle);
    forward(50);
    Serial.println("Mode normal - mengikuti dinding kanan");
  }

  debugSensors();
}

void updateSensors() {
  RangeDepan = depan.MeasureInCentimeters();
  RangeKanan = kanan.MeasureInCentimeters();
  RangeKiri  = kiri.MeasureInCentimeters();
}

void detectSignals() {
  detect_sig2 = false;
  detect_sig1 = false;
  x_sig2 = -1;
  x_sig1 = -1;

  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    uint16_t sig = pixy.ccc.blocks[i].m_signature;
    int x = pixy.ccc.blocks[i].m_x;
    if (sig == 2 && x_sig2 == -1) {
      detect_sig2 = true;
      x_sig2 = x;
    } else if (sig == 1 && x_sig1 == -1) {
      detect_sig1 = true;
      x_sig1 = x;
    }
  }
}

void updateAvoidLogic(unsigned long now) {
  if (!avoidingRight && detect_sig2 && x_sig2 >= 10 && RangeKanan > 5) {
    avoidingRight = true; avoidingLeft = false;
    currentMode = AVOID_RIGHT;
    Serial.println("SIG2 terdeteksi - menghindar ke kanan");
  }
  if (detect_sig2) lastSig2Seen = now;
  if (avoidingRight && !detect_sig2 && now - lastSig2Seen >= RECOVERY_DELAY && RangeKiri < 10) {
    avoidingRight = false;
    currentMode = NORMAL;
    Serial.println("SIG2 selesai - kembali ke jalur tengah");
  }

  if (!avoidingLeft && detect_sig1 && x_sig1 <= 150 && RangeKiri > 5) {
    avoidingLeft = true; avoidingRight = false;
    currentMode = AVOID_LEFT;
    leftAvoidStartTime = now;
    Serial.println("SIG1 terdeteksi - menghindar ke kiri");
  }
  if (detect_sig1) lastSig1Seen = now;
  if (avoidingLeft && !detect_sig1 && now - lastSig1Seen >= RECOVERY_DELAY && RangeKanan < 10) {
    avoidingLeft = false;
    leftAvoidDuration = now - leftAvoidStartTime;
    recoveryFromLeft = true;
    recoveryStartTime = now;
    currentMode = NORMAL;
    Serial.print("SIG1 selesai - recovery kanan selama ");
    Serial.print(leftAvoidDuration); Serial.println(" ms");
  }
}

void proportionalAvoid(int error_x, bool right) {
  int angle_offset = constrain(0.3 * error_x, 0, 30);
  int steer_angle = right ? (servo_center + angle_offset) : (servo_center - angle_offset);
  int speed = constrain(map(abs(error_x), 0, 150, 70, 40), 40, 70);
  steering.write(steer_angle);
  forward(speed);
  Serial.println(right ? "Hindar kanan proporsional" : "Hindar kiri proporsional");
}

void handleFrontAvoid(unsigned long now) {
  unsigned long elapsed = now - actionStartTime;
  if (elapsed < 2000) {
    steering.write(servo_center);
    backward(60);
    Serial.println("Mundur karena objek depan");
  } else if (elapsed < 5000) {
    steering.write(60);
    forward(60);
    Serial.println("Belok kiri setelah mundur");
  } else {
    currentMode = NORMAL;
    Serial.println("Selesai hindari depan - mode normal");
  }
}

void forward(int speed) {
  analogWrite(in1Pin, speed); analogWrite(in2Pin, 0);
  analogWrite(in3Pin, speed); analogWrite(in4Pin, 0);
}

void backward(int speed) {
  analogWrite(in1Pin, 0); analogWrite(in2Pin, speed);
  analogWrite(in3Pin, 0); analogWrite(in4Pin, speed);
}

void debugSensors() {
  Serial.print("Depan: "); Serial.print(RangeDepan); Serial.print(" cm | ");
  Serial.print("Kanan: "); Serial.print(RangeKanan); Serial.print(" cm | ");
  Serial.print("Kiri: ");  Serial.print(RangeKiri);  Serial.println(" cm");
}