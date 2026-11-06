#include "command_handler.h"
#include "gait_controller.h"
#include "servo_controller.h"

namespace cmd {

static char buf[32];
static uint8_t len = 0;

static void printHelp() {
  Serial.println(F(
    "\n--- Commands ---\n"
    " start / stop / stand / check\n"
    " fwd / back / left / right\n"
    " speed <-1..1> / turn <-1..1>\n"
    " status / help\n"));
}

static void printStatus() {
  Serial.print(gait::walking ? F("WALK") : F("STANDBY"));
  Serial.print(F("  speed=")); Serial.print(gait::speed, 2);
  Serial.print(F("  turn="));  Serial.print(gait::turn, 2);
  Serial.print(F("  phase=")); Serial.println(gait::phase, 3);
}

static void walk(float speed, float turn) {
  gait::setSpeed(speed);
  gait::setTurn(turn);
  if (!gait::walking) gait::start();
}

static bool is(const char* s, const char* name) { return strcasecmp(s, name) == 0; }

static void run(char* s) {
  while (*s == ' ') s++;
  if (!*s) return;

  if      (is(s, "start"))  gait::start();
  else if (is(s, "stop") || is(s, "stand")) gait::stop();
  else if (is(s, "check"))  { gait::stop(); servo::selfCheck(); }
  else if (is(s, "fwd"))    walk(1.0f, 0.0f);
  else if (is(s, "back"))   walk(-1.0f, 0.0f);
  else if (is(s, "left"))   walk(gait::speed, -0.8f);
  else if (is(s, "right"))  walk(gait::speed, 0.8f);
  else if (!strncasecmp(s, "speed", 5)) { gait::setSpeed(atof(s + 5)); printStatus(); }
  else if (!strncasecmp(s, "turn", 4))  { gait::setTurn(atof(s + 4));  printStatus(); }
  else if (is(s, "status")) printStatus();
  else if (is(s, "help") || is(s, "?")) printHelp();
  else {
    Serial.print(F("Unknown: "));
    Serial.println(s);
  }
}

void init() { printHelp(); }

void update() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      while (len && buf[len - 1] == ' ') len--;
      buf[len] = '\0';
      run(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

}
