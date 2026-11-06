"""歩容の安定性解析。実行: python sim/run_analysis.py"""
import os
from gait_core import *

ph = np.linspace(0, 1, 500, endpoint=False)
m = np.array([margin(p) for p in ph])
three = m[~np.isnan(m)]

print(f"CYCLE {CYCLE_SEC}s  YAW_AMP {YAW_AMP}°  KNEE_LIFT {KNEE_LIFT}°  "
      f"SWING_DUTY {SWING_DUTY}  BODY_TILT {BODY_TILT}°")
print(f"3脚支持 {len(three) / len(ph):.0%} / 4脚接地 {1 - len(three) / len(ph):.0%}")
print(f"安定余裕 min {three.min():.2f} mm / avg {three.mean():.2f} mm")

for i in range(4):
    start = -OFFSET[i] % 1.0
    print(f"{NAMES[i]}: 遊脚 {start:.2f} 〜 {(start + SWING_DUTY) % 1.0:.2f}")

print("ブラウザ版:", "file://" + os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_sim.html"))
