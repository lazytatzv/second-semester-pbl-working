#pragma once
#include <Arduino.h>
#include "config.h"

/**
 * @file servo_controller.h
 * @brief PCA9685 サーボドライバ低レベル制御
 */

class ServoController {
public:
  ServoController();

  // 初期化 (PCA9685のセットアップ)
  void init();

  // 指定チャンネルへの安全な角度出力 (0〜180度にクランプ)
  void writeServo(uint8_t channel, float angleDeg);

  // 全関節をニュートラル直立スタンバイ姿勢にセット
  void setStandbyPose();

  // 単一脚の角度をセット (yaw, knee)
  void setLegAngle(uint8_t legIdx, float yawDeg, float kneeDeg);

  // 全脚セルフチェック動作 (1本ずつ足を上げて正常稼働をテスト)
  void runSelfCheck();
};

extern ServoController servoCtrl;
