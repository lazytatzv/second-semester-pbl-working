#!/usr/bin/env python3
"""スライド用: 歩行フェーズ、関節軌道、足上げ運動学の3分割図を生成するスクリプト。
出力: docs/gait_overview.png
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 日本語フォント設定
plt.rcParams['font.family'] = 'Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus'] = False

fig, axes = plt.subplots(1, 3, figsize=(16, 5.2), gridspec_kw={'width_ratios': [1.1, 1.2, 1.1]})
fig.patch.set_facecolor('#ffffff')

# 共通カラー
cols = {"FL": "#d62728", "RR": "#ff7f0e", "FR": "#1f77b4", "RL": "#2ca02c"}
names = ["FL", "RR", "FR", "RL"]
offsets = [0.00, 0.25, 0.50, 0.75]  # 遊脚開始フェーズ

# ==============================================================================
# パネル 1: 歩行フェーズ (脚のタイミングと接地順)
# ==============================================================================
ax0 = axes[0]
ax0.set_facecolor('#f8f9fa')

for idx, (name, off) in enumerate(zip(names, offsets)):
    y = 3 - idx
    # 背景 (全域接地 = 薄いグレー)
    ax0.barh(y, 1.0, left=0.0, height=0.55, color="#e9ecef", edgecolor="#ced4da", lw=1.2)
    # 遊脚期間 (20% = カラーハイライト)
    ax0.barh(y, 0.20, left=off, height=0.55, color=cols[name], alpha=0.85, edgecolor="black", lw=1.5,
             label=f"{name} (空中)" if idx == 0 else "")
    # ラベル
    ax0.text(off + 0.10, y, f"遊脚 (空中)", ha="center", va="center", color="white", fontsize=9.5, fontweight="bold")

ax0.set_yticks([3, 2, 1, 0])
ax0.set_yticklabels(["1. FL (前左)", "2. RR (後右)", "3. FR (前右)", "4. RL (後左)"], fontsize=11, fontweight="bold")
ax0.set_xlim(0, 1.0)
ax0.set_xlabel("歩行サイクル位相 (0.0 〜 1.0) [1周期 = 2.5秒]", fontsize=10.5)
ax0.set_title("① 歩行フェーズ (対角クロール順)", fontsize=13, fontweight="bold", pad=12)

# ガイド線
for p in [0.25, 0.50, 0.75]:
    ax0.axvline(p, color="#adb5bd", ls="--", lw=1.0)
ax0.text(0.5, -0.7, "★ 常に「3本が接地(80%) ＋ 1本が空中(20%)」で安定", ha="center", fontsize=10.5,
         fontweight="bold", color="#198754", bbox=dict(boxstyle="round,pad=0.4", fc="#d1e7dd", ec="#badbcc"))

# ==============================================================================
# パネル 2: 1歩の関節軌道 (サイン波・コサイン波)
# ==============================================================================
ax1 = axes[1]
ax1.set_facecolor('#f8f9fa')

u = np.linspace(0, 1.0, 300)
yaw = np.zeros_like(u)
lift = np.zeros_like(u)

# 0〜0.2: 遊脚, 0.2〜1.0: 支持脚
sw = u < 0.20
s = u[sw] / 0.20
yaw[sw] = -np.cos(np.pi * s)
lift[sw] = np.sin(np.pi * s)

st = ~sw
tau = (u[st] - 0.20) / 0.80
yaw[st] = 1.0 - 2.0 * tau
lift[st] = 0.0

# プロット
l1, = ax1.plot(u, yaw, color="#0d6efd", lw=2.8, label="付け根 Yaw (前後スイング)")
l2, = ax1.plot(u, lift, color="#dc3545", lw=2.8, ls="--", label="ヒザ Knee (足上げリフト)")

# 領域の塗り分け
ax1.axvspan(0, 0.20, color="#cfe2ff", alpha=0.5, label="遊脚期 (空中 20%)")
ax1.axvspan(0.20, 1.0, color="#e2e3e5", alpha=0.4, label="支持脚期 (地面 80%)")

ax1.axhline(0, color="#adb5bd", lw=0.8, ls=":")
ax1.set_xlim(0, 1.0)
ax1.set_ylim(-1.25, 1.35)
ax1.set_xlabel("1脚の局所位相 (0.0 〜 1.0)", fontsize=10.5)
ax1.set_ylabel("正規化指令値 (-1.0 〜 +1.0)", fontsize=10.5)
ax1.set_title("② 関節の軌道生成 (サイン・コサイン波)", fontsize=13, fontweight="bold", pad=12)
ax1.legend(loc="upper right", fontsize=9.2, framealpha=0.9)

ax1.annotate("足を持ち上げ\n(sin波)", xy=(0.10, 1.0), xytext=(0.02, 1.15),
             arrowprops=dict(arrowstyle="->", color="#dc3545", lw=1.5), fontsize=9.5, fontweight="bold", color="#dc3545")
ax1.annotate("前へ素早く振る\n(-cos波)", xy=(0.10, 0.0), xytext=(0.22, 0.4),
             arrowprops=dict(arrowstyle="->", color="#0d6efd", lw=1.5), fontsize=9.5, fontweight="bold", color="#0d6efd")
ax1.annotate("地面を蹴って前進 (等速キック)", xy=(0.60, 0.0), xytext=(0.35, -0.85),
             arrowprops=dict(arrowstyle="->", color="#495057", lw=1.5), fontsize=9.5, fontweight="bold", color="#212529")

# ==============================================================================
# パネル 3: 運動学幾何図 (足上げ高さの計算)
# ==============================================================================
ax2 = axes[2]
ax2.set_facecolor('#f8f9fa')

L2 = 70.0  # スネ長 mm
th_ground = 25.0   # 接地角 deg
th_lift = 73.0     # 最高点角 deg (25 + 48)

# 膝関節を原点とする
knee_x, knee_y = 0.0, 75.0

# 接地脚の足先 (25°)
rad_g = np.radians(th_ground)
foot_g_x = knee_x + L2 * np.sin(rad_g)
foot_g_y = knee_y - L2 * np.cos(rad_g)

# 最高点の足先 (73°)
rad_l = np.radians(th_lift)
foot_l_x = knee_x + L2 * np.sin(rad_l)
foot_l_y = knee_y - L2 * np.cos(rad_l)

lift_h = foot_l_y - foot_g_y  # 42.97 mm

# 地面ライン
ax2.axhline(foot_g_y, color="#6c757d", lw=2.5, ls="-")
ax2.text(20, foot_g_y - 4.5, "地面 (Ground)", fontsize=9.5, color="#6c757d", fontweight="bold")

# スネの描画
ax2.plot([knee_x, foot_g_x], [knee_y, foot_g_y], color="#495057", lw=3.5, label="接地時 (25°)")
ax2.plot([knee_x, foot_l_x], [knee_y, foot_l_y], color="#dc3545", lw=3.5, label="最高点 (73°: +48°リフト)")
ax2.plot(knee_x, knee_y, "o", color="#212529", ms=10)
ax2.text(knee_x - 12, knee_y + 3, "ヒザ関節", fontsize=10, fontweight="bold")
ax2.plot(foot_g_x, foot_g_y, "o", color="#495057", ms=8)
ax2.plot(foot_l_x, foot_l_y, "o", color="#dc3545", ms=8)

# 角度円弧
arc_g = patches.Arc((knee_x, knee_y), 35, 35, angle=270, theta1=0, theta2=th_ground, color="#495057", lw=1.5)
arc_l = patches.Arc((knee_x, knee_y), 50, 50, angle=270, theta1=th_ground, theta2=th_lift, color="#dc3545", lw=2.0)
ax2.add_patch(arc_g); ax2.add_patch(arc_l)
ax2.text(knee_x + 5, knee_y - 25, "25°", fontsize=9, color="#495057", fontweight="bold")
ax2.text(knee_x + 18, knee_y - 22, "+48°\n(曲げ)", fontsize=9.5, color="#dc3545", fontweight="bold")

# 高さの寸法線
dim_x = 78.0
ax2.plot([foot_l_x + 5, dim_x + 5], [foot_l_y, foot_l_y], color="#dc3545", ls=":", lw=1.2)
ax2.plot([foot_g_x + 5, dim_x + 5], [foot_g_y, foot_g_y], color="#495057", ls=":", lw=1.2)
ax2.annotate("", xy=(dim_x, foot_l_y), xytext=(dim_x, foot_g_y),
             arrowprops=dict(arrowstyle="<->", color="#dc3545", lw=2.2))
ax2.text(dim_x + 4, (foot_l_y + foot_g_y) / 2, f"h = 約 43 mm\n(足上げ高さ)", color="#dc3545",
         va="center", fontsize=11, fontweight="bold")

ax2.set_xlim(-25, 115)
ax2.set_ylim(-5, 90)
ax2.set_aspect("equal")
ax2.axis("off")
ax2.set_title("③ 運動学 (足上げ高さの幾何計算)", fontsize=13, fontweight="bold", pad=12)

# 計算式ボックス
calc_text = "h = 70 × (cos 25° - cos 73°)\n  = 70 × (0.906 - 0.292) ≃ 43 mm"
ax2.text(45, 15, calc_text, ha="center", fontsize=10.2, fontweight="bold",
         color="#212529", bbox=dict(boxstyle="round,pad=0.5", fc="#fff3cd", ec="#ffe69c"))

plt.tight_layout()
out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "gait_overview.png")
plt.savefig(out_path, dpi=150, bbox_inches="tight")
print(f"Saved: {out_path}")
