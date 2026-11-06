#pragma once
#include <Arduino.h>
#include "config.h"

/**
 * @file gait_controller.h
 * @brief CPG歩容生成 & ロボット状態管理
 */

enum RobotState {
  STATE_STANDBY,  // 直立待機
  STATE_WALK,     // 歩行動作中
};

class GaitController {
public:
  GaitController();

  void init();

  // ループごとの歩行更新 (dtミリ秒進行)
  void update(uint16_t dtMs);

  // 状態変更コマンド
  void start();
  void stop();
  void stand();

  // 動作パラメータ変更
  void setSpeed(float speed); // -1.0(後退) 〜 +1.0(前進), 0(停止)
  void setTurn(float turn);   // -1.0(左旋回) 〜 +1.0(右旋回), 0(直進)

  // ゲッター
  RobotState getState() const { return state; }
  float getPhase() const { return phase; }
  float getSpeed() const { return speedCmd; }
  float getTurn() const { return turnCmd; }

private:
  RobotState state;
  float phase;      // 歩行位相 (0.0〜1.0)
  float speedCmd;   // 前後進速度指令
  float turnCmd;    // 旋回指令

  // 単一脚の角度計算
  void calculateLeg(uint8_t legIdx, float currentPhase, float rollTilt,
                    float &outYaw, float &outKnee);

  // 体重移動量(ロール)計算
  float calculateRollTilt(float currentPhase);
};

extern GaitController gaitCtrl;
