"""4脚ロボット 最新CPG歩行シミュレータ＆安定性解析スクリプト
実行: python sim/run_analysis.py
"""
import sys
import os
import numpy as np

# sim ディレクトリをパスに追加
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from gait_core import *

def analyze_gait():
    print("=" * 60)
    print("  4-Leg Spider Robot CPG Gait Analysis (main.cpp Ver 2.0)")
    print("=" * 60)
    print(f"  CYCLE_SEC:    {CYCLE_SEC:.1f} s")
    print(f"  YAW_AMP:      {YAW_AMP:.1f} deg")
    print(f"  KNEE_LIFT:    {KNEE_LIFT:.1f} deg")
    print(f"  SWING_DUTY:   {SWING_DUTY:.2f} (Stance Overlap: {(1-4*SWING_DUTY)*100:.0f}%)")
    print(f"  BODY_TILT:    {BODY_TILT:.1f} deg")
    print("-" * 60)

    ph = np.linspace(0, 1, 500, endpoint=False)
    margins = []
    speeds = []
    
    # 支持多角形と安定余裕のサンプリング
    three_leg_count = 0
    four_leg_count = 0

    for p in ph:
        m = margin(p)
        if not np.isnan(m):
            margins.append(m)
            three_leg_count += 1
        else:
            four_leg_count += 1

    min_m = min(margins) if margins else 0
    mean_m = np.mean(margins) if margins else 0

    print(f"【安定性解析結果】")
    print(f"  - 3脚支持フェーズ割合:   {three_leg_count / len(ph) * 100:.1f} %")
    print(f"  - 4脚接地フェーズ割合:   {four_leg_count / len(ph) * 100:.1f} % (転倒リセット期間)")
    print(f"  - 最小安定余裕 (Min Margin): {min_m:.2f} mm  (> 0: 常に静的安定)")
    print(f"  - 平均安定余裕 (Avg Margin): {mean_m:.2f} mm")

    print("-" * 60)
    print("【各脚の遊脚タイミング (Phase 0.00 〜 1.00)】")
    for i in range(4):
        # 遊脚が始まる位相
        p_start = (-OFFSET[i]) % 1.0
        p_end = (p_start + SWING_DUTY) % 1.0
        print(f"  - 脚 {NAMES[i]:2s}: Phase {p_start:.2f} 〜 {p_end:.2f} (遊脚) / 離地->前方スイング->着地")

    print("=" * 60)
    print("Webブラウザで動くリアルタイムシミュレータ:")
    print(f"  file://{os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot_sim.html')}")
    print("=" * 60)

if __name__ == "__main__":
    analyze_gait()
