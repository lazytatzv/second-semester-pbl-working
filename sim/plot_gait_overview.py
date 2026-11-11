#!/usr/bin/env python3
"""スライド・論文用: 学術的で端正なデザインの歩容・軌道・運動学概要図。
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
plt.rcParams['font.size'] = 10

fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.8), gridspec_kw={'width_ratios': [1.0, 1.15, 1.05]})
fig.patch.set_facecolor('#ffffff')

# ==============================================================================
# (a) 歩行位相シーケンス (対角クロール)
# ==============================================================================
ax0 = axes[0]
ax0.set_facecolor('#ffffff')

names = ["FL (前左)", "RR (後右)", "FR (前右)", "RL (後左)"]
offsets = [0.00, 0.25, 0.50, 0.75]

for idx, (name, off) in enumerate(zip(names, offsets)):
    y = 3 - idx
    # 支持脚期 (接地 80%) : 薄いグレー
    ax0.barh(y, 1.0, left=0.0, height=0.48, color="#f1f3f5", edgecolor="#868e96", lw=0.8)
    # 遊脚期 (空中 20%) : 濃紺
    ax0.barh(y, 0.20, left=off, height=0.48, color="#1c3d5a", edgecolor="#1c3d5a", lw=0.8)
    ax0.text(off + 0.10, y, "遊脚", ha="center", va="center", color="#ffffff", fontsize=9)

ax0.set_yticks([3, 2, 1, 0])
ax0.set_yticklabels(names, fontsize=9.5)
ax0.set_xlim(0, 1.0)
ax0.set_xticks([0.0, 0.25, 0.50, 0.75, 1.0])
ax0.set_xlabel("歩行位相 phi (1周期 T = 2.5 s)", fontsize=9.5)
ax0.set_title("(a) クロール歩容位相 (3脚支持率 80%)", fontsize=11, fontweight="bold", pad=10)
ax0.grid(axis="x", ls=":", color="#ced4da", lw=0.7)

# 凡例代わりの注釈
ax0.plot([], [], color="#1c3d5a", label="遊脚期 (Swing, 20%)")
ax0.plot([], [], color="#f1f3f5", markeredgecolor="#868e96", label="支持脚期 (Stance, 80%)")
ax0.legend(loc="upper right", fontsize=8.5, frameon=True, edgecolor="#ced4da")

# ==============================================================================
# (b) 関節角軌道 (1歩の正規化波形)
# ==============================================================================
ax1 = axes[1]
ax1.set_facecolor('#ffffff')

u = np.linspace(0, 1.0, 400)
yaw = np.zeros_like(u)
lift = np.zeros_like(u)

sw = u < 0.20
s = u[sw] / 0.20
yaw[sw] = -np.cos(np.pi * s)
lift[sw] = np.sin(np.pi * s)

st = ~sw
tau = (u[st] - 0.20) / 0.80
yaw[st] = 1.0 - 2.0 * tau
lift[st] = 0.0

ax1.plot(u, yaw, color="#0b4f6c", lw=2.0, label="付け根 Yaw (前後振り: -cos / 等速キック)")
ax1.plot(u, lift, color="#b02a37", lw=2.0, ls="--", label="膝 Knee (足上げ: sin / 接地維持)")

# 境界線
ax1.axvline(0.20, color="#495057", ls="--", lw=0.9)
ax1.text(0.10, -1.18, "遊脚 (空中)", ha="center", fontsize=8.5, color="#1c3d5a")
ax1.text(0.60, -1.18, "支持脚 (地面蹴り出し)", ha="center", fontsize=8.5, color="#495057")

ax1.axhline(0, color="#ced4da", lw=0.7, ls="-")
ax1.set_xlim(0, 1.0)
ax1.set_ylim(-1.3, 1.3)
ax1.set_xticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
ax1.set_xlabel("単脚の局所位相 u (0.0 〜 1.0)", fontsize=9.5)
ax1.set_ylabel("正規化関節角 theta / theta_max", fontsize=9.5)
ax1.set_title("(b) 単脚関節軌道 (滑らかな連続波形)", fontsize=11, fontweight="bold", pad=10)
ax1.legend(loc="upper right", fontsize=8.5, frameon=True, edgecolor="#ced4da")
ax1.grid(True, ls=":", color="#e9ecef", lw=0.7)

# ==============================================================================
# (c) リンク運動学 (足先クリアランス幾何)
# ==============================================================================
ax2 = axes[2]
ax2.set_facecolor('#ffffff')

L2 = 70.0        # スネ長 mm
th0 = 25.0       # 接地時開き角 deg
d_th = 48.0      # リフト角 deg
th1 = th0 + d_th # 最高点角 73 deg

knee = np.array([0.0, 75.0])

# 接地足先 (25°)
rad0 = np.radians(th0)
foot0 = knee + np.array([L2 * np.sin(rad0), -L2 * np.cos(rad0)])

# 最高点足先 (73°)
rad1 = np.radians(th1)
foot1 = knee + np.array([L2 * np.sin(rad1), -L2 * np.cos(rad1)])

dh = foot1[1] - foot0[1]  # 42.97 mm

# 地面
ground_y = foot0[1]
ax2.axhline(ground_y, color="#343a40", lw=1.2)
ax2.text(-15, ground_y - 4.0, "地面 (Ground)", fontsize=8.5, color="#343a40")

# 基準鉛直線
ax2.plot([knee[0], knee[0]], [ground_y, knee[1]], color="#adb5bd", ls=":", lw=1.0)

# リンク (スネ)
ax2.plot([knee[0], foot0[0]], [knee[1], foot0[1]], color="#495057", lw=2.2, label=f"接地時 (theta_0 = {th0:.0f}°)")
ax2.plot([knee[0], foot1[0]], [knee[1], foot1[1]], color="#b02a37", lw=2.2, label=f"最高点 (theta_1 = {th1:.0f}°)")

# 関節シンボル
ax2.plot(knee[0], knee[1], "o", color="#ffffff", markeredgecolor="#212529", markeredgewidth=1.8, ms=7)
ax2.text(knee[0] - 5, knee[1] + 3.5, "Knee (膝)", fontsize=8.5)
ax2.plot(foot0[0], foot0[1], "o", color="#495057", ms=5)
ax2.plot(foot1[0], foot1[1], "o", color="#b02a37", ms=5)

# 角度円弧
arc0 = patches.Arc(knee, 38, 38, angle=270, theta1=0, theta2=th0, color="#495057", lw=1.0)
arc1 = patches.Arc(knee, 52, 52, angle=270, theta1=th0, theta2=th1, color="#b02a37", lw=1.2)
ax2.add_patch(arc0); ax2.add_patch(arc1)
ax2.text(knee[0] + 5, knee[1] - 26, f"{th0:.0f}°", fontsize=8.5, color="#495057")
ax2.text(knee[0] + 18, knee[1] - 22, f"+{d_th:.0f}°", fontsize=8.5, color="#b02a37", fontweight="bold")

# スネ長さ注記
mid0 = (knee + foot0) / 2
ax2.text(mid0[0] - 14, mid0[1], f"L = {L2:.0f} mm", fontsize=8.0, color="#495057")

# 高さの寸法線
dim_x = 76.0
ax2.plot([foot1[0] + 3, dim_x + 3], [foot1[1], foot1[1]], color="#ced4da", ls="--", lw=0.8)
ax2.plot([foot0[0] + 3, dim_x + 3], [foot0[1], foot0[1]], color="#ced4da", ls="--", lw=0.8)
ax2.annotate("", xy=(dim_x, foot1[1]), xytext=(dim_x, foot0[1]),
             arrowprops=dict(arrowstyle="<->", color="#b02a37", lw=1.5))
ax2.text(dim_x + 3.5, (foot1[1] + foot0[1]) / 2, f"Delta h = {dh:.1f} mm\n(足上げクリアランス)",
         color="#b02a37", va="center", fontsize=8.5, fontweight="bold")

# 幾何式 (端正な数式表記)
calc_eq = r"$\Delta h = L_2 (\cos 25^\circ - \cos 73^\circ) \approx 43.0\ \mathrm{mm}$"
ax2.text(35, 12, calc_eq, ha="center", fontsize=9.0, color="#212529",
         bbox=dict(boxstyle="square,pad=0.4", fc="#ffffff", ec="#ced4da", lw=0.8))

ax2.set_xlim(-20, 115)
ax2.set_ylim(-8, 88)
ax2.set_aspect("equal")
ax2.axis("off")
ax2.set_title("(c) 膝関節の幾何と足上げ高さ", fontsize=11, fontweight="bold", pad=10)
ax2.legend(loc="upper left", fontsize=8.0, frameon=True, edgecolor="#ced4da")

plt.tight_layout()
out_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "gait_overview.png")
plt.savefig(out_path, dpi=180, bbox_inches="tight")
print(f"Saved: {out_path}")
