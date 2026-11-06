#pragma once
#include <Arduino.h>

struct LegConfig {
  const char* name;
  uint8_t chYaw, chKnee;   // PCA9685 チャンネル
  int8_t  yawDir, kneeDir; // 回転方向 (+1 / -1)
  float   yawTrim, kneeTrim;
  float   offset;          // 歩行位相オフセット (0〜1)
};

// 遊脚順: FL -> RR -> FR -> RL (対角クロール)
const LegConfig LEGS[4] = {
  // name chY chK  yDir kDir  yTrim kTrim offset
  { "FL", 0,  1,  +1,  +1,   0.0f, 0.0f, 0.00f },
  { "FR", 2,  3,  -1,  -1,   0.0f, 0.0f, 0.50f },
  { "RL", 4,  5,  +1,  +1,   0.0f, 0.0f, 0.25f },
  { "RR", 6,  7,  -1,  -1,   0.0f, 0.0f, 0.75f },
};

// 歩行パラメータ
const float CYCLE_SEC     = 2.5f;
const float YAW_AMP       = 24.0f; // 歩幅 [deg]
const float KNEE_STANCE   = 25.0f; // 立ち姿勢のスネ開き角 [deg]
const float KNEE_LIFT     = 28.0f; // 足上げ [deg]
const float SWING_DUTY    = 0.20f;
const float STANCE_SMOOTH = 0.50f; // 0:等速 〜 1:コサイン
const float BODY_TILT     = 4.0f;  // 体重移動 [deg]
const float SHIFT_DUTY    = 0.05f; // 遊脚の前後で体重移動を出し入れする位相幅

const uint16_t LOOP_DT_MS = 20;

// サーボ
const int   SERVO_MIN_PULSE = 120; // 0deg
const int   SERVO_MAX_PULSE = 520; // 180deg
const float NEUTRAL         = 90.0f;
const long  BAUD_RATE       = 115200L;
