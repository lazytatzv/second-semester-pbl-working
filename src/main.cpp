#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ======================================================
// PCA9685 / Servo settings
// ======================================================

#define SERVO_FREQ 50

// スクリーンショットの値を基準にする
#define SERVOMIN 150
#define SERVOMAX 375

const int NUM_SERVOS = 8;

const int CH[NUM_SERVOS] = {
    0, 1,     // 左前
    2, 3,     // 右前
    4, 5,     // 左後
    6, 7      // 右後
};


// ======================================================
// Leg number
// ======================================================

enum Leg {
    LF = 0,   // Left Front
    RF = 1,   // Right Front
    LR = 2,   // Left Rear
    RR = 3    // Right Rear
};


// ======================================================
// Servo position
// ======================================================

// スクリーンショットの中央付近の値
const int CENTER = 263;

// 前後方向の移動量
const int STEP_SIZE = 35;

// 脚を持ち上げる量
const int LIFT_SIZE = 80;


// ======================================================
// 歩行設定
// ======================================================

// 4脚が1回ずつ動く時間
const unsigned long GAIT_PERIOD = 2400;

// PCA9685への更新間隔
const unsigned long UPDATE_INTERVAL = 20;


// ======================================================
// 各脚の位相
// ======================================================
//
// 1周期の中で、各脚が動くタイミングをずらす
//
// 左前 → 右後 → 右前 → 左後
//
// 0.00
// 0.25
// 0.50
// 0.75
//

float phaseOffset[4] = {
    0.00,   // LF
    0.50,   // RF
    0.75,   // LR
    0.25    // RR
};


// ======================================================
// サーボの向きを設定
// ======================================================
//
// +1 : 通常
// -1 : 反転
//
// 実機に合わせて変更する
//

int direction[NUM_SERVOS] = {

     1,  1,    // LF
    -1,  1,    // RF
     1, -1,    // LR
    -1, -1     // RR
};


// ======================================================
// 初期姿勢
// ======================================================

int current[NUM_SERVOS] = {
    CENTER, CENTER,
    CENTER, CENTER,
    CENTER, CENTER,
    CENTER, CENTER
};


// ======================================================
// サーボ出力
// ======================================================

void setServo(int index, int value)
{
    value = constrain(value, SERVOMIN, SERVOMAX);

    pwm.setPWM(
        CH[index],
        0,
        value
    );

    current[index] = value;
}


// ======================================================
// 0～1の滑らかな補間
// ======================================================

float smoothStep(float x)
{
    x = constrain(x, 0.0f, 1.0f);

    return x * x * (3.0f - 2.0f * x);
}


// ======================================================
// 脚1本の動作を計算
// ======================================================
//
// phase:
// 0.0 ～ 1.0
//
// 0.0 ～ 0.25 : 脚を後ろから前へ移動
// 0.25～ 0.50 : 脚を下ろす
// 0.50～ 1.00 : 接地して後ろへ動かす
//
// ======================================================

void calculateLeg(
    int leg,
    float phase,
    int &joint1,
    int &joint2
)
{
    // ----------------------------------------------
    // 基本位置
    // ----------------------------------------------

    joint1 = CENTER;
    joint2 = CENTER;


    // ----------------------------------------------
    // 遊脚区間
    // ----------------------------------------------

    if (phase < 0.25f)
    {
        // 0～0.25
        // 脚を持ち上げながら前へ

        float t = phase / 0.25f;

        float s = smoothStep(t);

        // 前後方向
        joint1 =
            CENTER
            + STEP_SIZE
            - (int)(2.0f * STEP_SIZE * s);

        // 上方向
        joint2 =
            CENTER
            - (int)(LIFT_SIZE * s);
    }

    // ----------------------------------------------
    // 脚を下ろす
    // ----------------------------------------------

    else if (phase < 0.50f)
    {
        float t =
            (phase - 0.25f) / 0.25f;

        float s = smoothStep(t);

        joint1 = CENTER - STEP_SIZE;

        joint2 =
            CENTER
            - LIFT_SIZE
            + (int)(LIFT_SIZE * s);
    }

    // ----------------------------------------------
    // 接地して後ろへ
    // ----------------------------------------------

    else
    {
        float t =
            (phase - 0.50f) / 0.50f;

        float s = smoothStep(t);

        // 接地後、脚を後ろへ動かす
        joint1 =
            CENTER
            - STEP_SIZE
            + (int)(2.0f * STEP_SIZE * s);

        joint2 = CENTER;
    }


    // ----------------------------------------------
    // 左右・前後脚によるサーボ方向補正
    // ----------------------------------------------

    int servo1 = leg * 2;
    int servo2 = leg * 2 + 1;

    joint1 =
        CENTER
        + (joint1 - CENTER)
        * direction[servo1];

    joint2 =
        CENTER
        + (joint2 - CENTER)
        * direction[servo2];
}


// ======================================================
// 4脚歩行
// ======================================================

void updateGait()
{
    static unsigned long lastUpdate = 0;

    unsigned long now = millis();

    // 20msごとに更新
    if (now - lastUpdate < UPDATE_INTERVAL)
        return;

    lastUpdate = now;


    // ----------------------------------------------
    // 0～1の歩行周期
    // ----------------------------------------------

    float basePhase =
        (float)(now % GAIT_PERIOD)
        / (float)GAIT_PERIOD;


    // ----------------------------------------------
    // 4脚を計算
    // ----------------------------------------------

    for (int leg = 0; leg < 4; leg++)
    {
        // 脚ごとに位相をずらす
        float phase =
            basePhase
            + phaseOffset[leg];

        // 0～1に戻す
        if (phase >= 1.0f)
            phase -= 1.0f;

        int joint1;
        int joint2;

        calculateLeg(
            leg,
            phase,
            joint1,
            joint2
        );

        // ------------------------------------------
        // PCA9685へ送信
        // ------------------------------------------

        setServo(
            leg * 2,
            joint1
        );

        setServo(
            leg * 2 + 1,
            joint2
        );
    }
}


// ======================================================
// 初期姿勢
// ======================================================

void stand()
{
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        setServo(i, CENTER);
    }
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
    Serial.begin(115200);

    // I2C
    Wire.begin();

    // PCA9685
    pwm.begin();

    pwm.setPWMFreq(SERVO_FREQ);

    delay(500);

    // ----------------------------------------------
    // 初期姿勢
    // ----------------------------------------------

    stand();

    delay(1500);

    Serial.println("4-leg crawl gait start");
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
    updateGait();
}
