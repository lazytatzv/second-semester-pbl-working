"""歩容シミュレータ共通部(main.cpp と同じ式・機体形状)。gait_sim.py / world_sim.py から使う。
機体座標: +x が前進方向、上から見た図。パラメータは main.cpp と揃える。
"""
import os
import numpy as np

# ---- main.cpp と同じ値 ----
CYCLE_SEC = 2.5
YAW_AMP = 24.0      # deg
KNEE_LIFT = 28.0    # deg
SWING_DUTY = 0.20   # 4脚接地期間を確保してロバスト化
STANCE_SMOOTH = 0.50
BODY_TILT = 4.0     # deg 体重移動量
OFFSET = [0.00, 0.50, 0.25, 0.75]   # 0=前左 1=前右 2=後左 3=後右 / 遊脚順 FL→RR→FR→RL
NAMES = ["FL", "FR", "RL", "RR"]

# ---- 機体形状 (仮の値。実測に合わせて変更) ----
B = 40.0      # 付け根の位置 (±B, ±B) [mm]
L = 80.0      # 脚の水平リーチ [mm]
LIFT_H = 28.0 # 膝を最大に上げたときの足先の高さ [mm] (表示用)

# 付け根位置と、X配置のニュートラル方向(上から見て外向き45度)
BASE = np.array([[B, B], [B, -B], [-B, B], [-B, -B]])
NEUTRAL = np.radians([45, -45, 135, -135])
# 「前進方向へ振る」ときの yaw の物理的な向き: 左脚は角度減少、右脚は増加
FWD_SIGN = np.array([-1, +1, -1, +1])


def get_current_roll_tilt(phase):
    """main.cpp の loop() と同じ体重移動(ロール)計算"""
    for i in range(4):
        u = (phase + OFFSET[i]) % 1.0
        if u < SWING_DUTY:
            weight = np.sin(np.pi * (u / SWING_DUTY))
            tilt_dir = -1.0 if (i % 2 == 0) else +1.0
            return tilt_dir * BODY_TILT * weight
    return 0.0


def leg_state(phase, i, speed=1.0, turn=0.0):
    """main.cpp の calculateLegAngles と同じロジック"""
    u = (phase + OFFSET[i]) % 1.0
    if u < SWING_DUTY:
        s = u / SWING_DUTY
        norm_yaw = -np.cos(np.pi * s)
        norm_lift = np.sin(np.pi * s)
        swing = True
    else:
        s = (u - SWING_DUTY) / (1.0 - SWING_DUTY)
        norm_yaw = (1.0 - STANCE_SMOOTH) * (1.0 - 2.0 * s) + STANCE_SMOOTH * np.cos(np.pi * s)
        norm_lift = 0.0
        swing = False

    side_sign = +1.0 if (i % 2 == 0) else -1.0
    leg_drive = (speed + side_sign * turn) * norm_yaw
    return leg_drive, norm_lift, swing


def foot(phase, i, speed=1.0, turn=0.0):
    yaw, lift, swing = leg_state(phase, i, speed, turn)
    ang = NEUTRAL[i] + FWD_SIGN[i] * np.radians(YAW_AMP * yaw)
    xy = BASE[i] + L * np.array([np.cos(ang), np.sin(ang)])
    return xy, LIFT_H * lift, swing


def tri_margin(pts, p=np.zeros(2)):
    """点 p(重心)から支持三角形の各辺までの符号付き最短距離。正=内側。"""
    c = pts.mean(axis=0)
    m = 1e9
    for k in range(3):
        a, b = pts[k], pts[(k + 1) % 3]
        e = b - a
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        if np.dot(n, c - a) < 0:
            n = -n
        m = min(m, np.dot(n, p - a))
    return m


def margin(phase):
    # 足先が接地(高さ<2mm)している脚を支持脚とみなす。3本のときだけ評価
    # ロール傾斜による重心移動 (H=40mm, dy = H * sin(roll))
    H = 40.0
    roll_deg = get_current_roll_tilt(phase)
    com = np.array([0.0, H * np.sin(np.radians(roll_deg))])

    pts = np.array([foot(phase, i)[0] for i in range(4) if foot(phase, i)[1] <= 2.0])
    return tri_margin(pts, com) if len(pts) == 3 else np.nan
