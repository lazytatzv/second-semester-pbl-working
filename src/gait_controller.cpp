#include "gait_controller.h"
#include "servo_controller.h"

GaitController gaitCtrl;

GaitController::GaitController()
  : state(STATE_STANDBY), phase(0.0f), speedCmd(1.0f), turnCmd(0.0f) {}

void GaitController::init() {
  state = STATE_STANDBY;
  phase = 0.0f;
  speedCmd = 1.0f;
  turnCmd = 0.0f;
  servoCtrl.setStandbyPose();
}

void GaitController::start() {
  state = STATE_WALK;
  Serial.println(F("[Gait] Walking STARTED"));
}

void GaitController::stop() {
  state = STATE_STANDBY;
  servoCtrl.setStandbyPose();
  Serial.println(F("[Gait] Walking STOPPED (Standing by)"));
}

void GaitController::stand() {
  state = STATE_STANDBY;
  phase = 0.0f;
  servoCtrl.setStandbyPose();
  Serial.println(F("[Gait] Stood in Neutral Pose"));
}

void GaitController::setSpeed(float speed) {
  speedCmd = constrain(speed, -1.0f, 1.0f);
  Serial.print(F("[Gait] Speed set to: "));
  Serial.println(speedCmd);
}

void GaitController::setTurn(float turn) {
  turnCmd = constrain(turn, -1.0f, 1.0f);
  Serial.print(F("[Gait] Turn set to: "));
  Serial.println(turnCmd);
}

float GaitController::calculateRollTilt(float currentPhase) {
  // 遊脚中の脚の反対側へ体重を逃がす
  for (uint8_t i = 0; i < 4; i++) {
    float u = currentPhase + LEGS[i].offset;
    if (u >= 1.0f) u -= 1.0f;
    if (u < DEFAULT_SWING_DUTY) {
      float weight = sin(PI * (u / DEFAULT_SWING_DUTY));
      float tiltDir = (i % 2 == 0) ? -1.0f : +1.0f;
      return tiltDir * DEFAULT_BODY_TILT * weight;
    }
  }
  return 0.0f;
}

void GaitController::calculateLeg(uint8_t legIdx, float currentPhase, float rollTilt,
                                 float &outYaw, float &outKnee) {
  const LegConfig &leg = LEGS[legIdx];

  float u = currentPhase + leg.offset;
  if (u >= 1.0f) u -= 1.0f;

  float normYaw = 0.0f;
  float normLift = 0.0f;

  if (u < DEFAULT_SWING_DUTY) {
    // 遊脚フェーズ: 前方へスイング
    float s = u / DEFAULT_SWING_DUTY;
    normYaw  = -cos(PI * s);
    normLift =  sin(PI * s);
  } else {
    // 接地フェーズ: 地面を蹴る
    float s = (u - DEFAULT_SWING_DUTY) / (1.0f - DEFAULT_SWING_DUTY);
    normYaw  = (1.0f - DEFAULT_STANCE_SMOOTH) * (1.0f - 2.0f * s) + DEFAULT_STANCE_SMOOTH * cos(PI * s);
    normLift = 0.0f;
  }

  // 旋回差動合成
  float sideSign = (legIdx % 2 == 0) ? +1.0f : -1.0f;
  float legDrive = (speedCmd + sideSign * turnCmd) * normYaw;

  // ロール補正
  float legTilt = sideSign * rollTilt;

  // 目標角度
  outYaw  = SERVO_NEUTRAL + leg.yawDir  * (DEFAULT_YAW_AMP   * legDrive);
  outKnee = SERVO_NEUTRAL + leg.kneeDir * (DEFAULT_KNEE_LIFT * normLift + legTilt);
}

void GaitController::update(uint16_t dtMs) {
  if (state != STATE_WALK) {
    return;
  }

  // 1. 位相の更新
  phase += (dtMs / 1000.0f) / DEFAULT_CYCLE_SEC;
  if (phase >= 1.0f) {
    phase -= 1.0f;
  }

  // 2. 体重移動(ロール)計算
  float rollTilt = calculateRollTilt(phase);

  // 3. 各脚サーボへの出力
  for (uint8_t i = 0; i < 4; i++) {
    float yaw, knee;
    calculateLeg(i, phase, rollTilt, yaw, knee);
    servoCtrl.setLegAngle(i, yaw, knee);
  }
}
