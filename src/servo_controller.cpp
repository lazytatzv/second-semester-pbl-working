#include "servo_controller.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
ServoController servoCtrl;

ServoController::ServoController() {}

void ServoController::init() {
  pwm.begin();
  pwm.setPWMFreq(50); // アナログサーボ標準 50Hz
  setStandbyPose();
}

void ServoController::writeServo(uint8_t channel, float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0f, 180.0f);
  int pulse = map((long)(angleDeg * 10), 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  pwm.setPWM(channel, 0, pulse);
}

void ServoController::setStandbyPose() {
  for (uint8_t i = 0; i < 4; i++) {
    setLegAngle(i, SERVO_NEUTRAL, SERVO_NEUTRAL);
  }
}

void ServoController::setLegAngle(uint8_t legIdx, float yawDeg, float kneeDeg) {
  if (legIdx >= 4) return;
  const LegConfig &leg = LEGS[legIdx];
  writeServo(leg.chYaw,  yawDeg  + leg.yawTrim);
  writeServo(leg.chKnee, kneeDeg + leg.kneeTrim);
}

void ServoController::runSelfCheck() {
  Serial.println(F("\n--- Starting Servo Self-Check ---"));
  setStandbyPose();
  delay(500);

  for (uint8_t i = 0; i < 4; i++) {
    const LegConfig &leg = LEGS[i];
    Serial.print(F("Testing Leg ["));
    Serial.print(leg.name);
    Serial.print(F("] (Yaw CH:"));
    Serial.print(leg.chYaw);
    Serial.print(F(", Knee CH:"));
    Serial.print(leg.chKnee);
    Serial.println(F(")..."));

    // 1. 膝を持ち上げる
    float liftedKnee = SERVO_NEUTRAL + leg.kneeDir * 35.0f;
    setLegAngle(i, SERVO_NEUTRAL, liftedKnee);
    delay(400);

    // 2. 付け根(Yaw)を前後に振る
    float fwdYaw = SERVO_NEUTRAL + leg.yawDir * 25.0f;
    setLegAngle(i, fwdYaw, liftedKnee);
    delay(300);

    float backYaw = SERVO_NEUTRAL - leg.yawDir * 25.0f;
    setLegAngle(i, backYaw, liftedKnee);
    delay(300);

    // 3. 元の位置へ着地
    setLegAngle(i, SERVO_NEUTRAL, SERVO_NEUTRAL);
    delay(300);
  }

  Serial.println(F("--- Self-Check Complete! Standing by. ---\n"));
  setStandbyPose();
}
