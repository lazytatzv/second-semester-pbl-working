#include "servo_controller.h"
#include <Adafruit_PWMServoDriver.h>

static Adafruit_PWMServoDriver pwm;

static void write(uint8_t ch, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  pwm.setPWM(ch, 0, map((long)(deg * 10), 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
}

namespace servo {

void init() {
  pwm.begin();
  pwm.setPWMFreq(50);
  standby();
}

void setLeg(uint8_t i, float yawDeg, float kneeDeg) {
  const LegConfig& leg = LEGS[i];
  write(leg.chYaw,  yawDeg  + leg.yawTrim);
  write(leg.chKnee, kneeDeg + leg.kneeTrim);
}

void standby() {
  for (uint8_t i = 0; i < 4; i++) {
    setLeg(i, NEUTRAL, NEUTRAL + LEGS[i].kneeDir * KNEE_STANCE);
  }
}

// 1本ずつ足を上げて前後に振る
void selfCheck() {
  Serial.println(F("--- Self-check ---"));
  standby();
  delay(500);

  for (uint8_t i = 0; i < 4; i++) {
    const LegConfig& leg = LEGS[i];
    Serial.print(F("Leg "));
    Serial.println(leg.name);

    float stanceKnee = NEUTRAL + leg.kneeDir * KNEE_STANCE;
    float liftKnee   = stanceKnee + leg.kneeDir * 35.0f;
    setLeg(i, NEUTRAL, liftKnee);                       delay(400);
    setLeg(i, NEUTRAL + leg.yawDir * 25.0f, liftKnee);  delay(300);
    setLeg(i, NEUTRAL - leg.yawDir * 25.0f, liftKnee);  delay(300);
    setLeg(i, NEUTRAL, stanceKnee);                     delay(300);
  }
  Serial.println(F("--- Done ---"));
}

}
