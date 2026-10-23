"""長時間(数周期)の俯瞰シミュレーション。ワールド座標で胴体の進行と足跡を描く。
実行: python sim/world_sim.py -> sim/world.png, sim/world.gif
接地脚の足先はワールドで固定(滑りなし)と仮定し、胴体速度を接地脚の足先の
機体座標での動きから求める(接地脚の平均)。姿勢(向き)は変化しない近似。
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from gait_core import *

CYCLES = 6
DT = 0.01
N = int(CYCLES * CYCLE_SEC / DT)
cols = ["tab:red", "tab:blue", "tab:green", "tab:orange"]

pos = np.zeros((N, 2))
feet_b = np.zeros((N, 4, 2))
stance = np.zeros((N, 4), bool)
mg = np.full(N, np.nan)
for n in range(N):
    p = (n * DT / CYCLE_SEC) % 1.0
    for i in range(4):
        xy, z, _ = foot(p, i)
        feet_b[n, i] = xy
        stance[n, i] = z <= 2.0
    mg[n] = margin(p)
    if n > 0:
        both = stance[n] & stance[n - 1]
        if both.any():
            d = (feet_b[n, both] - feet_b[n - 1, both]).mean(axis=0)
            pos[n] = pos[n - 1] - d      # 接地足先は動かない → 胴体が逆向きに動く
        else:
            pos[n] = pos[n - 1]
feet_w = pos[:, None, :] + feet_b

# 足跡 = 遊脚→接地に切り替わった点
steps = [[] for _ in range(4)]
for n in range(1, N):
    for i in range(4):
        if stance[n, i] and not stance[n - 1, i]:
            steps[i].append(feet_w[n, i])

t = np.arange(N) * DT
speed = np.gradient(pos[:, 0], DT)
print("胴体速度 min/max [mm/s]:", round(speed.min(),1), round(speed.max(),1), " 横方向 y範囲:", round(pos[:,1].min(),1), round(pos[:,1].max(),1))
print(f"{CYCLES}周期({CYCLES*CYCLE_SEC:.0f}s)で前進 {pos[-1,0]:.0f} mm, 横ずれ {pos[-1,1]:.1f} mm, "
      f"平均速度 {pos[-1,0]/t[-1]:.1f} mm/s, 最小安定余裕 {np.nanmin(mg):.1f} mm")

out = os.path.dirname(os.path.abspath(__file__))
fig, ax = plt.subplots(2, 1, figsize=(15, 8), gridspec_kw={"height_ratios": [2.2, 1]})
a = ax[0]
a.plot(pos[:, 0], pos[:, 1], "k-", lw=1.5, label="body center")
for i in range(4):
    s = np.array(steps[i])
    if len(s):
        a.plot(s[:, 0], s[:, 1], "o", color=cols[i], label=NAMES[i] + " footfall")
for n in range(0, N, int(CYCLE_SEC / DT / 2)):
    c = pos[n]
    a.add_patch(plt.Rectangle(c - B, 2 * B, 2 * B, fill=False, ec="gray", alpha=.5))
a.set_aspect("equal"); a.legend(ncol=5, loc="upper left")
a.set_title(f"World view: {CYCLES} cycles, advance {pos[-1,0]:.0f} mm, lateral drift {pos[-1,1]:.1f} mm")
a.set_xlabel("x [mm]"); a.set_ylabel("y [mm]")
a = ax[1]
a.plot(t, speed, label="body speed x [mm/s]")
a.plot(t, mg, label="stability margin [mm]")
a.axhline(0, color="r", ls="--"); a.legend(); a.set_xlabel("time [s]")
plt.tight_layout(); plt.savefig(os.path.join(out, "world.png"), dpi=100)

# アニメーション(俯瞰・全体固定ビュー)
fig, a = plt.subplots(figsize=(11, 4))
a.set_xlim(-150, pos[:, 0].max() + 160); a.set_ylim(-170, 170); a.set_aspect("equal")
a.plot(pos[:, 0], pos[:, 1], "k:", lw=.8)
rect = a.add_patch(plt.Rectangle((0, 0), 2 * B, 2 * B, fill=False))
legl = [a.plot([], [], "-", color=cols[i], lw=2)[0] for i in range(4)]
dots = [a.plot([], [], "o", color=cols[i], ms=8)[0] for i in range(4)]
prints = [a.plot([], [], ".", color=cols[i], ms=4)[0] for i in range(4)]
ttl = a.set_title("")
STEP = 4
def upd(f):
    n = f * STEP
    rect.set_xy(pos[n] - B)
    for i in range(4):
        c = BASE[i] + pos[n]
        legl[i].set_data([c[0], feet_w[n, i, 0]], [c[1], feet_w[n, i, 1]])
        dots[i].set_data([feet_w[n, i, 0]], [feet_w[n, i, 1]])
        dots[i].set_markerfacecolor(cols[i] if stance[n, i] else "none")
        s = np.array([q for q in steps[i] if q[0] <= feet_w[n, i, 0] + 1e-9]) if steps[i] else np.empty((0, 2))
        if len(s): prints[i].set_data(s[:, 0], s[:, 1])
    ttl.set_text(f"t={t[n]:.1f}s  x={pos[n,0]:.0f}mm  (hollow=swing)")
    return []
FuncAnimation(fig, upd, frames=N // STEP, blit=False).save(
    os.path.join(out, "world.gif"), writer=PillowWriter(fps=25))
