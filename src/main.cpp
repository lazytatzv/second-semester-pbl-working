// 2DOF x 4脚 蜘蛛型ロボット: 最小CPG(位相オシレータ)実装
// 各脚: 付け根=yaw, 膝=pitch。上から見てX配置。フィードバック無し(オープンループ)。
// サーボ角 90deg = 各関節のニュートラル。
// 脚の「ニュートラルの向き」(45/135/225/315deg)は機構側で決まるので、
// ソフトでは「ニュートラルからの振れ」だけを出力する。
//
// 歩容: クロール(ウォーク)。常に3脚が接地し、1脚ずつ遊脚にする。
// 遊脚順は 前左→後右→前右→後左(対角順)。左右が完全に鏡像(半周期ずれ)で、全脚が同じ動きをする。
// 重心移動の補正はしない(機体が十分安定している前提)。胴体の動きは滑らか。
// (sim/gait_sim.py, sim/world_sim.py で軌道を確認できる。機体寸法は仮値)
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

// ---- 調整パラメータ ----
const float CYCLE_SEC = 3.0f;   // 1周期 [s] (遅いほど安定。慣れたら短く)
const float YAW_AMP   = 25.0f;  // yaw振幅 [deg]
const float KNEE_LIFT = 25.0f;  // 遊脚時の膝の持ち上げ量 [deg]
const float SWING_DUTY = 0.25f; // 1周期のうち遊脚の割合 (4脚で1脚ずつ → 0.25)
const float STANCE_SMOOTH = 0.5f; // 接地中のyaw: 0=等速(胴体の蛇行が小さい) 1=コサイン(離地/着地の速度が滑らか)
const float DT_MS     = 20;     // 制御周期

// 脚番号: 0=前左 1=前右 2=後左 3=後右 (上から見てXの各角)
// 各脚の位相オフセット(周期の割合): 遊脚順 0→3→1→2 (位相が遅れるほど後に遊脚)
const float OFFSET[4]   = {0.00f, 0.50f, 0.25f, 0.75f};
// 左右で取り付けが鏡像なので yaw / knee の回転方向を反転
const int   YAW_DIR[4]  = {+1, -1, +1, -1};  // 実機で符号確認
const int   KNEE_DIR[4] = {+1, -1, +1, -1};
// 個体差トリム [deg] (90からのオフセット)
const float YAW_TRIM[4]  = {0, 0, 0, 0};
const float KNEE_TRIM[4] = {0, 0, 0, 0};

// PCA9685 チャンネル: {yaw, knee}
const uint8_t CH[4][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};

const int SERVOMIN = 110, SERVOMAX = 510;  // 0〜180deg に対応するパルス(要校正)

void setServo(uint8_t ch, float deg) {
  deg = constrain(deg, 0, 180);
  pwm.setPWM(ch, 0, map((long)(deg * 10), 0, 1800, SERVOMIN, SERVOMAX));
}

float phase = 0;           // CPG状態は位相1つだけ (0..1)
float direction = 1.0f;    // +1前進 / -1後退 / 0停止(振幅スケールとして使用)

void setup() {
  Serial.begin(115200);
  Serial.println("Spider CPG init!");
  pwm.begin();
  pwm.setPWMFreq(50);
  for (int i = 0; i < 4; i++) {
    setServo(CH[i][0], 90 + YAW_TRIM[i]);
    setServo(CH[i][1], 90 + KNEE_TRIM[i]);
  }
  delay(1000);
}

void loop() {
  phase += DT_MS / 1000.0f / CYCLE_SEC;
  if (phase >= 1.0f) phase -= 1.0f;

  for (int i = 0; i < 4; i++) {
    float u = phase + OFFSET[i];
    if (u >= 1.0f) u -= 1.0f;

    float yaw, lift;
    if (u < SWING_DUTY) {
      // 遊脚: 持ち上げながら後端→前端へ素早く戻す(コサインで滑らかに)
      float s = u / SWING_DUTY;                       // 0..1
      yaw  = -cos(PI * s);                            // -1 → +1
      lift = sin(PI * s);                             // 0 → 1 → 0
    } else {
      // 接地: 前端→後端へ蹴る(等速とコサインのブレンド)
      float s = (u - SWING_DUTY) / (1.0f - SWING_DUTY);  // 0..1
      yaw  = (1 - STANCE_SMOOTH) * (1.0f - 2.0f * s) + STANCE_SMOOTH * cos(PI * s);  // +1 → -1
      lift = 0;
    }

    setServo(CH[i][0], 90 + YAW_TRIM[i]  + YAW_DIR[i]  * YAW_AMP * yaw * direction);
    setServo(CH[i][1], 90 + KNEE_TRIM[i] + KNEE_DIR[i] * KNEE_LIFT * lift);
  }
  delay(DT_MS);
}
