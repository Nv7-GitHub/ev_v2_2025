#include "mt6701_header.hpp"
#include <MPU6050.h>

MT6701 leftEnc;
MT6701 rightEnc;
MPU6050 mpu;

float prevLeft = 0;
float prevRight = 0;
long start = 0;
unsigned long prevTime = 0;
long lastPrint = 0;

void setup() {
  Serial.begin(9600);

  // Setup I2C devices
  setupWires();
  if (!rightEnc.initializeI2C(true)) {
    while(1) {
      Serial.println("Failed to initailize right enc");
    }
  }
  if (!leftEnc.initializeI2C(false)) {
    while(1) {
      Serial.println("Failed to initailize left enc");
    }
  }

  mpu.initialize();
  mpu.setZGyroOffset(-3);
  if (!mpu.testConnection()) {
    while(1) {
      Serial.println("Failed to initailize IMU");
    }
  }

  // LED
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);

  // Motor PWM pins
  pinMode(18, OUTPUT); // Left
  pinMode(28, OUTPUT);
  analogWriteFreq(10000);
  analogWriteRange(10000);
  analogWrite(18, 10000);
  analogWrite(28, 10000);

  // Direction pins
  pinMode(19, OUTPUT); // Left
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  digitalWrite(19, LOW);

  // Wait for GO button to be pressed
  pinMode(2, INPUT_PULLUP);
  while (digitalRead(2) != LOW) { // Wait for button press
    Serial.println("Connected!");
    delay(50);
  }
  while (digitalRead(2) != HIGH) { // Wait for button release
    digitalWrite(25, LOW);

    // Set initial position to be 0
    prevLeft = leftEnc.angleRead();
    prevRight = rightEnc.angleRead();
  }

  // Initialize everything before going
  digitalWrite(25, HIGH);
  start = millis();
  prevTime = micros();
}

float distAxial = 0;
float distLateral = 0;
float theta = 0;

const float targetDist = 7;
const float mPerDeg = (1.0f/360.0f) * 2 * PI * 0.0254;
const float P_forward = 0.8;
const float P_difference = 2.0;
const float D = 0.15;
const float maxSpeed = 1.0;
const float trackWidth = 150.0/1000.0; // mm converted to m

// Handles wrap-around in encoder reading (since its absolute pos)
float normalizeDelta(float delta) {
  if (delta < -180.0f) {
    delta += 360.0;
  } else if (delta > 180.0f) {
    delta -= 360.0;
  }
  return fmod(delta, 180);
}

bool done = false;
long doneTime = 0;
long loopCnt = 0;
float filteredOmega = 0;

void loop() {
  // If finished, don't do anything
  if (done) {
    motorWriteL(0.0);
    motorWriteR(0.0);
    digitalWrite(25, LOW);
    Serial.print("Done! axialDist:");
    Serial.print(distAxial);
    Serial.print(",lateralDist:");
    Serial.print(distLateral);
    Serial.print(",time:");
    float time = ((float)doneTime)/1000.0f;
    Serial.print(time);
    Serial.print(",loopRate:");
    Serial.println(((float)loopCnt)/time);
    delay(50);
    return;
  }

  loopCnt++; // So that I can print loop rate!

  // Encoders
  float left = leftEnc.angleRead();
  float right = rightEnc.angleRead();
  float deltaL = normalizeDelta(prevLeft - left);
  float deltaR = normalizeDelta(right - prevRight);

  // Calculate heading
  //float dTheta = (deltaR - deltaL)/trackWidth * mPerDeg; // Encoder-based angle calculation
  // IMU-based angle calculation
  float dT = (micros() - prevTime)/1000000.0f;
  float omega = (mpu.getRotationZ()/131.0f) * DEG_TO_RAD;
  float dTheta = dT * omega;
  prevTime = micros();
  theta += dTheta;

  // Make it wrap-around
  while (theta > PI) {
    theta -= 2*PI;
  }
  while (theta < -PI) {
    theta += 2*PI;
  }

  // Calculate odometry
  float distAvg = (deltaL + deltaR)/2.0f * mPerDeg;
  distAxial += distAvg * cos(theta);
  distLateral += distAvg * sin(theta);
  
  // Controllers
  float distRem = (targetDist - distAxial);
  if (abs(distRem) < 0.003 && (abs(deltaL) + abs(deltaR)) < 5) { // 0.3cm
    doneTime = millis() - start;
    done = true;
    return;
  }

  // Forward PID controller
  float pow;
  if (abs(distRem) > 0.02) { // 1 cm
    pow = P_forward * (targetDist - distAxial);
  } else {
    pow = P_forward * 3 * (targetDist - distAxial);
  }
  if (pow > 0) { // Friction compensation
    pow += 0.1;
  } else {
    pow -= 0.1;
  }

  pow *= min(((float)(millis() - start))/(1000.0f), 1.0); // Accel curve for 500ms
  
  // Make sure it follows max speed
  if (pow > maxSpeed) {
    pow = maxSpeed;
  } else if (pow < -maxSpeed) {
    pow = -maxSpeed;
  }

  // Calculate target angle
  float ang;
  if (distRem > 0.4) {
    ang = theta - atan2(0-distLateral, targetDist - distAxial);
  } else {
    ang = theta;
  }

  // Calculate W
  float w = ang * P_difference;

  // D term
  filteredOmega = filteredOmega * 0.3 + omega * 0.7; // Low-pass filter
  w += filteredOmega * D;

  // Apply W
  float powL = pow + w;
  float powR = pow - w;

  // Scale so that they don't pass 1
  if (abs(powL) > 1.0 || abs(powR) > 1.0) {
    if (abs(powL) > abs(powR)) {
      powL /= abs(powL);
      powR /= abs(powL);
    } else {
      powL /= abs(powR);
      powR /= abs(powR);
    }
  }

  // Powers
  motorWriteL(powL);
  motorWriteR(powR);

  // Debugging
  if (Serial && millis() - lastPrint > 50) {
    Serial.print("time:");
    Serial.print(millis() - start);
    /*Serial.print(",leftDelta:");
    Serial.print(deltaL);
    Serial.print(",rightDelta:");
    Serial.print(deltaR); 
    Serial.print(",left:");
    Serial.print(leftEnc.angleRead());
    Serial.print(",right:");
    Serial.print(rightEnc.angleRead());
    Serial.print(",leftPrev:");
    Serial.print(prevLeft);
    Serial.print(",rightPrev:");
    Serial.print(prevRight);*/
    Serial.print(",distLateral:");
    Serial.print(distLateral);
    Serial.print(",distAxial:");
    Serial.print(distAxial);
    Serial.print(",theta:");
    Serial.print(theta * RAD_TO_DEG);
    Serial.print(",ang:");
    Serial.print(ang * RAD_TO_DEG);
    Serial.print(",powL:");
    Serial.print(powL);
    Serial.print(",powR:");
    Serial.println(powR);
    lastPrint = millis();
  }
  prevLeft = left;
  prevRight = right;

  //delay(1);
}

void motorWriteR(double speed) {
  if (speed > 0) {
    analogWrite(28, (1-speed)*10000.0f);
    digitalWrite(22, LOW);
  } else {
    analogWrite(28, (1+speed)*10000.0f);
    digitalWrite(22, HIGH);
  }
}
void motorWriteL(double speed) {
  if (speed > 0) {
    analogWrite(18, (1-speed)*10000.0f);
    digitalWrite(19, HIGH);
  } else {
    analogWrite(18, (1+speed)*10000.0f);
    digitalWrite(19, LOW);
  }
}
