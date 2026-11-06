/**
 * @file main.cpp
 * @brief 2自由度 x 4脚 蜘蛛型ロボット: メインプログラム
 *
 * 【モジュール構成】
 * - include/config.h          : ハードウェアピン、トリム、歩行定数
 * - include/servo_controller.h : PCA9685制御、安全クリップ、セルフチェック
 * - include/gait_controller.h  : CPG歩容アルゴリズム、状態管理 (WALK/STANDBY)
 * - include/command_handler.h : USBシリアル通信コマンドインタフェース
 */

#include <Arduino.h>
#include "config.h"
#include "servo_controller.h"
#include "gait_controller.h"
#include "command_handler.h"

void setup() {
  // 1. USBシリアル通信の開始
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 2000); // USB接続待機 (Unoでは即抜ける)

  Serial.println(F("\n=============================================="));
  Serial.println(F("  🕷️ 4-Leg Spider Robot Initializing..."));
  Serial.println(F("=============================================="));

  // 2. サーボドライバ (PCA9685) の初期化 & ニュートラル立ち
  servoCtrl.init();

  // 3. 歩行コントローラ初期化
  gaitCtrl.init();

  // 4. シリアルコマンドハンドラ初期化 (ヘルプメニュー表示)
  cmdHandler.init();

  Serial.println(F("Initialization complete. Ready for commands!"));
}

void loop() {
  // 1. USBシリアルからのコマンド受信処理
  cmdHandler.update();

  // 2. 歩行状態に応じた関節角度計算 & サーボ更新
  gaitCtrl.update(LOOP_DT_MS);

  // 3. 定周期待機 (50Hz)
  delay(LOOP_DT_MS);
}
