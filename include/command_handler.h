#pragma once
#include <Arduino.h>

/**
 * @file command_handler.h
 * @brief USBシリアル通信コマンドパーサ & ディスパッチャ
 */

class CommandHandler {
public:
  CommandHandler();

  void init();

  // シリアル受信をポーリング処理 (loop内で呼ぶ)
  void update();

  // ヘルプ表示
  void printHelp();

  // 現在ステータス表示
  void printStatus();

private:
  static const uint8_t BUF_SIZE = 32;
  char rxBuffer[BUF_SIZE];
  uint8_t rxIndex;

  void processCommand(char* cmd);
};

extern CommandHandler cmdHandler;
