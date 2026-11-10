#include "gait_controller.h"
#include "servo_controller.h"

namespace gait {

bool  walking = false;
float phase = 0.0f;
float speed = 1.0f;
float turn  = 0.0f;

// 左脚は+1、右脚は-1
static float side(uint8_t i) { return i % 2 == 0 ? 1.0f : -1.0f; }

static float legPhase(uint8_t i) {
  float u = phase + LEGS[i].offset;
  return u >= 1.0f ? u - 1.0f : u;
}

// 遊脚中は 1.0、その前後 SHIFT_DUTY で滑らかにコサインで出し入れする重み
static float shiftWeight(float u) {
  if (u < SWING_DUTY) return 1.0f;
  if (SHIFT_DUTY <= 0.0f) return 0.0f;
  if (u < SWING_DUTY + SHIFT_DUTY) return 0.5f * (1.0f + cos(PI * (u - SWING_DUTY) / SHIFT_DUTY));
  if (u >= 1.0f - SHIFT_DUTY) return 0.5f * (1.0f - cos(PI * (u - (1.0f - SHIFT_DUTY)) / SHIFT_DUTY));
  return 0.0f;
}

// 遊脚の反対側へ体を傾ける。足を上げる前に傾け終わっておく
static float rollTilt() {
  if (BODY_TILT <= 0.0f) return 0.0f;
  float t = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    t -= side(i) * shiftWeight(legPhase(i));
  }
  return BODY_TILT * constrain(t, -1.0f, 1.0f);
}

void start() {
  walking = true;
  Serial.println(F("[Gait] start"));
}

void stop() {
  walking = false;
  phase = 0.0f;
  servo::standby();
  Serial.println(F("[Gait] stop"));
}

void setSpeed(float v) { speed = constrain(v, -1.0f, 1.0f); }
void setTurn(float v)  { turn  = constrain(v, -1.0f, 1.0f); }

void update(uint16_t dtMs) {
  if (!walking) return;

  phase += dtMs / 1000.0f / CYCLE_SEC;
  if (phase >= 1.0f) phase -= 1.0f;

  float tilt = rollTilt();

  for (uint8_t i = 0; i < 4; i++) {
    float u = legPhase(i);

    float yaw, lift = 0.0f;
    if (u < SWING_DUTY) {
      float s = u / SWING_DUTY;
      yaw  = -cos(PI * s);
      lift =  sin(PI * s);
    } else {
      float s = (u - SWING_DUTY) / (1.0f - SWING_DUTY);
      yaw = 1.0f - 2.0f * s;
    }

    float drive = (speed + side(i) * turn) * yaw;
    servo::setLeg(i,
      NEUTRAL + LEGS[i].yawDir  * YAW_AMP * drive,
      NEUTRAL + LEGS[i].kneeDir * (KNEE_STANCE + KNEE_LIFT * lift + side(i) * tilt));
  }
}

}
