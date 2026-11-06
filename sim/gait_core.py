"""歩容シミュレータ共通部 (gait_controller.cpp と同じ式・パラメータ)。
機体座標: +x が前進方向、上から見た図。
"""
import os
import numpy as np

CYCLE_SEC = 2.5
YAW_AMP = 24.0
KNEE_STANCE = 25.0  # 立ち姿勢のスネ開き角 [deg]
KNEE_LIFT = 48.0    # 足上げ [deg] (約43mm上がる)
SWING_DUTY = 0.20
STANCE_SMOOTH = 0.50
BODY_TILT = 4.0
SHIFT_DUTY = 0.05  # 遊脚の前後で体重移動を出し入れする位相幅
OFFSET = [0.00, 0.50, 0.25, 0.75]
NAMES = ["FL", "FR", "RL", "RR"]

# 機体形状 [mm] (実機寸法: 車体幅150mm, hip->knee 60mm, knee->足先 70mm)
B = 75.0       # 付け根位置 (±B, ±B) ※前後長も150mmと仮定
L1 = 60.0      # hip -> knee [mm]
L2 = 70.0      # knee -> 足先 [mm]
L = L1 + L2 * np.sin(np.radians(KNEE_STANCE))  # 立ち姿勢での水平リーチ (約89.6mm)
COM_H = L2 * np.cos(np.radians(KNEE_STANCE))   # 立ち姿勢での膝・重心高さ (約63.4mm)
LIFT_H = L2 * (np.cos(np.radians(KNEE_STANCE)) - np.cos(np.radians(KNEE_STANCE + KNEE_LIFT)))  # 最大足上げ高さ (約21.3mm)

BASE = np.array([[B, B], [B, -B], [-B, B], [-B, -B]])
NEUTRAL = np.radians([45, -45, 135, -135])
FWD_SIGN = np.array([-1, +1, -1, +1])  # 前へ振るときの yaw の向き


def side(i):
    return 1.0 if i % 2 == 0 else -1.0


def shift_weight(u):
    """遊脚中は 1、その前後 SHIFT_DUTY でコサインで出し入れする重み"""
    if u < SWING_DUTY:
        return 1.0
    if SHIFT_DUTY <= 0:
        return 0.0
    if u < SWING_DUTY + SHIFT_DUTY:
        return 0.5 * (1 + np.cos(np.pi * (u - SWING_DUTY) / SHIFT_DUTY))
    if u >= 1.0 - SHIFT_DUTY:
        return 0.5 * (1 - np.cos(np.pi * (u - (1.0 - SHIFT_DUTY)) / SHIFT_DUTY))
    return 0.0


def roll_tilt(phase):
    """遊脚の反対側へ体を傾ける。足を上げる前に傾け終わっておく"""
    t = sum(-side(i) * shift_weight((phase + OFFSET[i]) % 1.0) for i in range(4))
    return BODY_TILT * float(np.clip(t, -1.0, 1.0))


def leg_state(phase, i, speed=1.0, turn=0.0):
    u = (phase + OFFSET[i]) % 1.0
    swing = u < SWING_DUTY
    if swing:
        s = u / SWING_DUTY
        yaw, lift = -np.cos(np.pi * s), np.sin(np.pi * s)
    else:
        s = (u - SWING_DUTY) / (1.0 - SWING_DUTY)
        yaw = (1.0 - STANCE_SMOOTH) * (1.0 - 2.0 * s) + STANCE_SMOOTH * np.cos(np.pi * s)
        lift = 0.0
    return (speed + side(i) * turn) * yaw, lift, swing


def foot(phase, i, speed=1.0, turn=0.0):
    yaw, lift, swing = leg_state(phase, i, speed, turn)
    ang = NEUTRAL[i] + FWD_SIGN[i] * np.radians(YAW_AMP * yaw)
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
    com = np.array([0.0, COM_H * np.sin(np.radians(roll_tilt(phase)))])
    feet = [foot(phase, i) for i in range(4)]
    pts = np.array([xy for xy, z, _ in feet if z <= 2.0])
    return tri_margin(pts, com) if len(pts) == 3 else np.nan
