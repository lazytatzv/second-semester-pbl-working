#pragma once
#include "config.h"

namespace servo {
void init();
void setLeg(uint8_t leg, float yawDeg, float kneeDeg);
void standby();
void selfCheck();
}
