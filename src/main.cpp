// 2自由度 x 4脚 蜘蛛型ロボット
#include <Arduino.h>
#include "config.h"
#include "servo_controller.h"
#include "gait_controller.h"
#include "command_handler.h"

void setup() {
  Serial.begin(BAUD_RATE);
  servo::init();
  cmd::init();
}

void loop() {
  cmd::update();
  gait::update(LOOP_DT_MS);
  delay(LOOP_DT_MS);
}
