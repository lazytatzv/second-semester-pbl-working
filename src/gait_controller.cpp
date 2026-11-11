#include "gait_controller.h"
#include "servo_controller.h"

namespace gait {

bool  walking = false;
float phase = 0.0f;
float speed = 1.0f;
float turn  = 0.0f;

// 前脚は+1、後脚は-1
static float xSign(uint8_t i) { return (i == 0 || i == 1) ? 1.0f : -1.0f; }

// 左脚は+1、右脚は-1
static float ySign(uint8_t i) { return (i % 2 == 0) ? 1.0f : -1.0f; }

static float legPhase(uint8_t i) {
  float u = phase + LEGS[i].offset;
  return u >= 1.0f ? u - 1.0f : u;
}

// 歩行サイクル全体を通して滑らかに連続して重心を回すヨー角補正 (全歩で均等にスライド)
static float yawBodyShift(uint8_t i, float p) {
  float ix = xSign(i);
  float iy = ySign(i);

  // X方向 (前後): 周期0.5 (2倍周波数) の連続波
  float xs = -cos(4.0f * PI * (p - 0.10f));
  // Y方向 (左右): 周期1.0 (1倍周波数) の連続波 (振幅係数 sqrt(2) ≒ 1.4142)
  float ys = 1.4142f * cos(2.0f * PI * (p - 0.475f));

  return -BODY_SHIFT_X * xs + (ix * iy) * (BODY_SHIFT_Y * ys);
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

    float yawShift = yawBodyShift(i, phase);
    float drive = (speed + ySign(i) * turn) * yaw;
    servo::setLeg(i,
      NEUTRAL + LEGS[i].yawDir  * (YAW_AMP * drive + yawShift),
      NEUTRAL + LEGS[i].kneeDir * (KNEE_STANCE + KNEE_LIFT * lift));
  }
}

}
