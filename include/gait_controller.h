#pragma once
#include "config.h"

namespace gait {
extern bool  walking;
extern float phase; // 0〜1
extern float speed; // -1(後退) 〜 +1(前進)
extern float turn;  // -1(左) 〜 +1(右)

void start();
void stop();
void setSpeed(float v);
void setTurn(float v);
void update(uint16_t dtMs);
}
