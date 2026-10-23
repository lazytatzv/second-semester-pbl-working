"""歩容シミュレータ共通部(main.cpp と同じ式・機体形状)。gait_sim.py / world_sim.py から使う。
機体座標: +x が前進方向、上から見た図。パラメータは main.cpp と揃える。
"""
import os
import numpy as np

# ---- main.cpp と同じ値 ----
CYCLE_SEC = 3.0
YAW_AMP = 25.0      # deg
KNEE_LIFT = 25.0    # deg
SWING_DUTY = 0.25
STANCE_SMOOTH = 0.5   # 接地中の yaw: 0=等速(胴体の蛇行が小さい) 1=コサイン(離地/着地の速度が滑らか)
OFFSET = [0.00, 0.50, 0.25, 0.75]   # 0=前左 1=前右 2=後左 3=後右 / 遊脚順 FL→RR→FR→RL
NAMES = ["FL", "FR", "RL", "RR"]

# ---- 機体形状 (仮の値。実測に合わせて変更) ----
B = 40.0      # 付け根の位置 (±B, ±B) [mm]
L = 80.0      # 脚の水平リーチ [mm]
LIFT_H = 25.0 # 膝を最大に上げたときの足先の高さ [mm] (表示用)

# 付け根位置と、X配置のニュートラル方向(上から見て外向き45度)
BASE = np.array([[B, B], [B, -B], [-B, B], [-B, -B]])
NEUTRAL = np.radians([45, -45, 135, -135])
# 「前進方向へ振る」ときの yaw の物理的な向き: 左脚は角度減少、右脚は増加
FWD_SIGN = np.array([-1, +1, -1, +1])


def leg_state(phase, i):
    """main.cpp の loop() と同じ。yaw(-1..1, +1=前端), lift(0..1), 遊脚か"""
    u = (phase + OFFSET[i]) % 1.0
    if u < SWING_DUTY:
        s = u / SWING_DUTY
        return -np.cos(np.pi * s), np.sin(np.pi * s), True
    s = (u - SWING_DUTY) / (1 - SWING_DUTY)
    return (1 - STANCE_SMOOTH) * (1 - 2 * s) + STANCE_SMOOTH * np.cos(np.pi * s), 0.0, False


def foot(phase, i):
    yaw, lift, swing = leg_state(phase, i)
    ang = NEUTRAL[i] + FWD_SIGN[i] * np.radians(YAW_AMP * yaw)
    xy = BASE[i] + L * np.array([np.cos(ang), np.sin(ang)])
    return xy, LIFT_H * lift, swing


def tri_margin(pts, p=np.zeros(2)):
    """点 p(重心=原点)から支持三角形の各辺までの符号付き最短距離。正=内側。"""
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
    # 足先がほぼ接地(高さ<2mm)している脚を支持脚とみなす。3本のときだけ評価
    pts = np.array([foot(phase, i)[0] for i in range(4) if foot(phase, i)[1] <= 2.0])
    return tri_margin(pts) if len(pts) == 3 else np.nan   # 4脚接地の瞬間は除外


