"""main.cpp の歩容(クロール)の軌道シミュレータ(機体座標)。
実行: python sim/gait_sim.py   -> sim/gait.png, sim/gait.gif を出力
パラメータ・歩容式は gait_core.py。
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from gait_core import *

ph = np.linspace(0, 1, 400, endpoint=False)
traj = np.array([[foot(p, i)[0] for i in range(4)] for p in ph])    # (N,4,2)
zs = np.array([[foot(p, i)[1] for i in range(4)] for p in ph])
mg = np.array([margin(p) for p in ph])
yaws = np.array([[leg_state(p, i)[0] * YAW_AMP for i in range(4)] for p in ph])
cols = ["tab:red", "tab:blue", "tab:green", "tab:orange"]

# ---- 静止画 ----
fig, ax = plt.subplots(1, 3, figsize=(17, 5.5))
a = ax[0]
for i in range(4):
    a.plot(traj[:, i, 0], traj[:, i, 1], color=cols[i], label=NAMES[i])
    a.plot(*BASE[i], "ks")
    a.plot([BASE[i][0], traj[0, i, 0]], [BASE[i][1], traj[0, i, 1]], color=cols[i], alpha=.3)
a.plot(0, 0, "k+", ms=12)
a.add_patch(plt.Rectangle((-B, -B), 2 * B, 2 * B, fill=False, ec="gray"))
a.annotate("", xy=(2 * L, 0), xytext=(L, 0), arrowprops=dict(arrowstyle="->"))
a.text(L * 1.1, 6, "forward")
a.set_aspect("equal"); a.legend(); a.set_title("Foot paths, top view (body frame)")
a.set_xlabel("x [mm]"); a.set_ylabel("y [mm]")

a = ax[1]
t = ph * CYCLE_SEC
for i in range(4):
    a.plot(t, yaws[:, i] + 60 * (3 - i), color=cols[i], label=f"{NAMES[i]} yaw")
    a.plot(t, zs[:, i] / LIFT_H * 15 + 60 * (3 - i) - 30, color=cols[i], ls="--")
a.set_yticks([60 * (3 - i) for i in range(4)]); a.set_yticklabels(NAMES)
a.set_title("yaw (solid) / lift (dashed) per leg"); a.set_xlabel("time [s]")

a = ax[2]
a.plot(t, mg); a.axhline(0, color="r", ls="--")
a.set_title(f"Stability margin (COM at origin)  min={np.nanmin(mg):.1f} mm")
a.set_xlabel("time [s]"); a.set_ylabel("distance to support edge [mm] (>0 = stable)")
plt.tight_layout()
out = os.path.dirname(os.path.abspath(__file__))
plt.savefig(os.path.join(out, "gait.png"), dpi=110)

# ---- アニメーション ----
fig, a = plt.subplots(figsize=(6, 6))
a.set_xlim(-60, 2.3 * L); a.set_ylim(-1.7 * L, 1.7 * L); a.set_aspect("equal")
body = a.add_patch(plt.Rectangle((-B, -B), 2 * B, 2 * B, fill=False))
a.plot(0, 0, "k+")
legs = [a.plot([], [], "-", color=cols[i], lw=2)[0] for i in range(4)]
dots = [a.plot([], [], "o", color=cols[i], ms=9)[0] for i in range(4)]
tri = a.fill([], [], alpha=.15, color="green")[0]
title = a.set_title("")


def upd(f):
    p = f / 100
    pts = []
    for i in range(4):
        xy, z, sw = foot(p, i)
        legs[i].set_data([BASE[i][0], xy[0]], [BASE[i][1], xy[1]])
        dots[i].set_data([xy[0]], [xy[1]])
        dots[i].set_markersize(9 + z / 2.5)
        dots[i].set_markerfacecolor("none" if sw else cols[i])
        if not sw:
            pts.append(xy)
    tri.set_xy(np.array(pts))
    title.set_text(f"t={p*CYCLE_SEC:.2f}s  margin={margin(p):.1f}mm (hollow=swing)")
    return legs + dots + [tri, title]


FuncAnimation(plt.gcf(), upd, frames=100, blit=False).save(
    os.path.join(out, "gait.gif"), writer=PillowWriter(fps=20))
print("min stability margin [mm]:", round(np.nanmin(mg), 1))
