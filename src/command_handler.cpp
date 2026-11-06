#include "command_handler.h"
#include "gait_controller.h"
#include "servo_controller.h"

CommandHandler cmdHandler;

CommandHandler::CommandHandler() : rxIndex(0) {
  memset(rxBuffer, 0, sizeof(rxBuffer));
}

void CommandHandler::init() {
  rxIndex = 0;
  printHelp();
}

void CommandHandler::printHelp() {
  Serial.println(F("\n========================================"));
  Serial.println(F(" 🕷️ 4-Leg Spider Robot Command Menu"));
  Serial.println(F("========================================"));
  Serial.println(F(" [Motion Commands]"));
  Serial.println(F("  start       : Start walking forward"));
  Serial.println(F("  stop        : Stop walking (standby)"));
  Serial.println(F("  stand       : Reset to neutral stand pose"));
  Serial.println(F("  check       : Run servo self-check (all legs)"));
  Serial.println(F(" [Direction & Speed]"));
  Serial.println(F("  fwd         : Set direction Forward"));
  Serial.println(F("  back        : Set direction Backward"));
  Serial.println(F("  left        : Turn Left"));
  Serial.println(F("  right       : Turn Right"));
  Serial.println(F("  speed <val> : Set speed (-1.0 to 1.0)"));
  Serial.println(F("  turn <val>  : Set turn (-1.0 to 1.0)"));
  Serial.println(F(" [Info]"));
  Serial.println(F("  status      : Print current robot state"));
  Serial.println(F("  help / ?    : Show this help"));
  Serial.println(F("========================================\n"));
}

void CommandHandler::printStatus() {
  Serial.println(F("\n--- Robot Telemetry ---"));
  Serial.print(F(" State: "));
  Serial.println(gaitCtrl.getState() == STATE_WALK ? F("WALKING") : F("STANDBY"));
  Serial.print(F(" Speed: "));
  Serial.println(gaitCtrl.getSpeed(), 2);
  Serial.print(F(" Turn : "));
  Serial.println(gaitCtrl.getTurn(), 2);
  Serial.print(F(" Phase: "));
  Serial.println(gaitCtrl.getPhase(), 3);
  Serial.println(F("-----------------------\n"));
}

void CommandHandler::processCommand(char* cmd) {
  // 前後の空白を除去
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  int len = strlen(cmd);
  while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\r' || cmd[len - 1] == '\n')) {
    cmd[--len] = '\0';
  }
  if (len == 0) return;

  // コマンド判定
  if (strcasecmp(cmd, "start") == 0) {
    gaitCtrl.start();
  }
  else if (strcasecmp(cmd, "stop") == 0) {
    gaitCtrl.stop();
  }
  else if (strcasecmp(cmd, "stand") == 0) {
    gaitCtrl.stand();
  }
  else if (strcasecmp(cmd, "check") == 0) {
    gaitCtrl.stop();
    servoCtrl.runSelfCheck();
  }
  else if (strcasecmp(cmd, "fwd") == 0) {
    gaitCtrl.setSpeed(1.0f);
    gaitCtrl.setTurn(0.0f);
    if (gaitCtrl.getState() != STATE_WALK) gaitCtrl.start();
  }
  else if (strcasecmp(cmd, "back") == 0) {
    gaitCtrl.setSpeed(-1.0f);
    gaitCtrl.setTurn(0.0f);
    if (gaitCtrl.getState() != STATE_WALK) gaitCtrl.start();
  }
  else if (strcasecmp(cmd, "left") == 0) {
    gaitCtrl.setTurn(-0.8f);
    if (gaitCtrl.getState() != STATE_WALK) gaitCtrl.start();
  }
  else if (strcasecmp(cmd, "right") == 0) {
    gaitCtrl.setTurn(0.8f);
    if (gaitCtrl.getState() != STATE_WALK) gaitCtrl.start();
  }
  else if (strncasecmp(cmd, "speed", 5) == 0) {
    float val = atof(cmd + 5);
    gaitCtrl.setSpeed(val);
  }
  else if (strncasecmp(cmd, "turn", 4) == 0) {
    float val = atof(cmd + 4);
    gaitCtrl.setTurn(val);
  }
  else if (strcasecmp(cmd, "status") == 0) {
    printStatus();
  }
  else if (strcasecmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
  }
  else {
    Serial.print(F("Unknown command: '"));
    Serial.print(cmd);
    Serial.println(F("'. Type 'help' for command list."));
  }
}

void CommandHandler::update() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';
        processCommand(rxBuffer);
        rxIndex = 0;
      }
    } else {
      if (rxIndex < BUF_SIZE - 1) {
        rxBuffer[rxIndex++] = c;
      } else {
        // バッファオーバーフロー防止
        rxIndex = 0;
      }
    }
  }
}
