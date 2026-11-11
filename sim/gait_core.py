"""歩容シミュレータ共通部 (gait_controller.cpp と同じ式・パラメータ)。
機体座標: +x が前進方向、上から見た図。
"""
import numpy as np

CYCLE_SEC = 2.5
YAW_AMP = 24.0
KNEE_STANCE = 25.0  # 立ち姿勢のスネ開き角 [deg]
KNEE_LIFT = 48.0    # 足上げ [deg] (約43mm上がる)
SWING_DUTY = 0.20
BODY_SHIFT_X = 4.0  # 前後重心スライド量 [deg]
BODY_SHIFT_Y = 7.0  # 左右重心スライド量 [deg]
COM_BIAS_X = -10.0  # 実機の重心オフセット [mm] (マイナスで後方寄り)
OFFSET = [0.00, 0.50, 0.25, 0.75]
NAMES = ["FL", "FR", "RL", "RR"]

# 機体形状 [mm] (実機寸法: 車体150x150mm, hip->knee 60mm, knee->足先 70mm)
B = 75.0       # 付け根位置 (±B, ±B)
L1 = 60.0      # hip -> knee [mm]
L2 = 70.0      # knee -> 足先 [mm]
L = L1 + L2 * np.sin(np.radians(KNEE_STANCE))  # 立ち姿勢での水平リーチ (約89.6mm)
COM_H = L2 * np.cos(np.radians(KNEE_STANCE))   # 立ち姿勢での膝・重心高さ (約63.4mm)
LIFT_H = L2 * (np.cos(np.radians(KNEE_STANCE)) - np.cos(np.radians(KNEE_STANCE + KNEE_LIFT)))  # 最大足上げ高さ (約43.0mm)

BASE = np.array([[B, B], [B, -B], [-B, B], [-B, -B]])
NEUTRAL = np.radians([45, -45, 135, -135])
FWD_SIGN = np.array([-1, +1, -1, +1])  # 前へ振るときの yaw の向き


def side(i):
    return 1.0 if i % 2 == 0 else -1.0


def x_sign(i):
    return 1.0 if i in [0, 1] else -1.0


def y_sign(i):
    return side(i)


def yaw_body_shift(i, phase):
    ix = x_sign(i)
    iy = y_sign(i)
    xs = -np.cos(4.0 * np.pi * (phase - 0.10))
    ys = 1.4142 * np.cos(2.0 * np.pi * (phase - 0.475))
    return -BODY_SHIFT_X * xs + (ix * iy) * (BODY_SHIFT_Y * ys)


def leg_state(phase, i, speed=1.0, turn=0.0):
    u = (phase + OFFSET[i]) % 1.0
    swing = u < SWING_DUTY
    if swing:
        s = u / SWING_DUTY
        yaw, lift = -np.cos(np.pi * s), np.sin(np.pi * s)
    else:
        s = (u - SWING_DUTY) / (1.0 - SWING_DUTY)
        yaw, lift = 1.0 - 2.0 * s, 0.0
    return (speed + side(i) * turn) * yaw, lift, swing


def foot(phase, i, speed=1.0, turn=0.0):
    yaw, lift, swing = leg_state(phase, i, speed, turn)
    yaw_shift = yaw_body_shift(i, phase)
    ang = NEUTRAL[i] + FWD_SIGN[i] * np.radians(YAW_AMP * yaw + yaw_shift)
    knee_deg = KNEE_STANCE + KNEE_LIFT * lift
    reach = L1 + L2 * np.sin(np.radians(knee_deg))
    z = L2 * (np.cos(np.radians(KNEE_STANCE)) - np.cos(np.radians(knee_deg)))
    xy = BASE[i] + reach * np.array([np.cos(ang), np.sin(ang)])
    return xy, z, swing


def tri_margin(pts, p=np.zeros(2)):
    """点 p から支持三角形の各辺までの符号付き最短距離 (正=内側)"""
    c = pts.mean(axis=0)
    m = np.inf
    for k in range(3):
        a, b = pts[k], pts[(k + 1) % 3]
        e = b - a
        n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        if np.dot(n, c - a) < 0:
            n = -n
        m = min(m, np.dot(n, p - a))
    return m


def margin(phase):
    """3脚支持のときの安定余裕。4脚接地中は nan"""
    feet = [foot(phase, i) for i in range(4)]
    pts = np.array([xy for xy, z, _ in feet if z <= 2.0])
    com = np.array([COM_BIAS_X, 0.0])
    return tri_margin(pts, com) if len(pts) == 3 else np.nan
