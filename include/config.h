#pragma once
#include <Arduino.h>

/**
 * @file config.h
 * @brief ロボット設定、ピン配置、歩行パラメータの定義
 */

// =============================================================================
// 1. 各脚の設定構造体 & テーブル
// =============================================================================

struct LegConfig {
  const char* name;   // 脚の名称 (FL:前左, FR:前右, RL:後左, RR:後右)
  uint8_t chYaw;      // PCA9685 チャンネル (付け根 yaw)
  uint8_t chKnee;     // PCA9685 チャンネル (膝 knee)
  int8_t  yawDir;     // 回転方向 (+1 または -1)
  int8_t  kneeDir;    // 回転方向 (+1 または -1)
  float   yawTrim;    // 付け根トリムオフセット [deg] (90deg基準)
  float   kneeTrim;   // 膝トリムオフセット   [deg] (90deg基準)
  float   offset;     // 歩行位相オフセット (0.0〜1.0)
};

// 4脚の設定テーブル (脚ごとの調整・配線変更はここを一箇所編集するだけで完了)
// 遊脚順序: FL(0.00) -> RR(0.25) -> FR(0.50) -> RL(0.75) (対角順クロール)
const LegConfig LEGS[4] = {
  // 名前,  chYaw, chKnee, yawDir, kneeDir, yawTrim, kneeTrim, offset
  { "FL",      0,      1,     +1,     +1,      0.0f,     0.0f,   0.00f }, // 前左 (0)
  { "FR",      2,      3,     -1,     -1,      0.0f,     0.0f,   0.50f }, // 前右 (1)
  { "RL",      4,      5,     +1,     +1,      0.0f,     0.0f,   0.25f }, // 後左 (2)
  { "RR",      6,      7,     -1,     -1,      0.0f,     0.0f,   0.75f }, // 後右 (3)
};

// =============================================================================
// 2. 歩行・チューニングパラメータ
// =============================================================================

const float DEFAULT_CYCLE_SEC   = 2.5f;   // 1周期の時間 [秒]
const float DEFAULT_YAW_AMP     = 24.0f;  // 付け根(yaw)の振幅 [deg] (歩幅)
const float DEFAULT_KNEE_LIFT   = 28.0f;  // 遊脚時の膝持ち上げ量 [deg] (足上げ高さ)
const float DEFAULT_SWING_DUTY  = 0.20f;  // 遊脚比率 (0.20 = 4脚接地期間を20%確保)
const float DEFAULT_STANCE_SMOOTH = 0.50f;// 接地中の平滑度 (0:等速 〜 1:コサイン)
const float DEFAULT_BODY_TILT   = 4.0f;   // 体重移動量 [deg] (ロール傾斜補正)

const uint16_t LOOP_DT_MS       = 20;     // 制御周期 [ms] (50Hz)

// =============================================================================
// 3. サーボハードウェアパラメータ
// =============================================================================

const int   SERVO_MIN_PULSE     = 110;    // 0deg相当のPWMパルス
const int   SERVO_MAX_PULSE     = 510;    // 180deg相当のPWMパルス
const float SERVO_NEUTRAL       = 90.0f;  // 関節基準角度 [deg]
const long  SERIAL_BAUD_RATE    = 115200L;// USBシリアル通信ボーレート
