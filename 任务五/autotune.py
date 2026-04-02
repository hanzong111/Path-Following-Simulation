"""
任务5：自动调参 — Hill Climbing
算法：从随机起点出发，逐步向更好的方向爬坡，直到无法改进为止
闭环：调参 → 仿真（headless） → 评估 → 爬坡
"""

from Algo import random_restart_hill_climbing
from Save_Json import save_results_autotune

# =============================================
# ✅ 修改这里
# =============================================
WAYPOINTS = [(0.0, 0.0), (4, 1), (8, -1)]

# 每个参数的 [最小值, 最大值, 步长]
PARAM_BOUNDS = {
    "lookahead": [2,  3, 0.1],
    "max_speed": [1,  2, 0.1],
    "min_speed": [0.5,  1.5, 0.1],
}

MAX_SIM_STEPS  = 10000
ARRIVE_THRESH  = 0.3
RESTARTS       = 5
# =============================================

# ──────────────────────────────────────────
# 主程序
# ──────────────────────────────────────────
def main():
    best_params, best_score, all_results = random_restart_hill_climbing(
        WAYPOINTS, PARAM_BOUNDS, restarts=RESTARTS
    )

    save_results_autotune(best_params, best_score, all_results)

    print("\n" + "="*55)
    print("  🏆 Hill Climbing 调参完成")
    print("="*55)
    print(f"  最优 score : {best_score:.4f}")
    print(f"  lookahead  : {best_params['lookahead']}")
    print(f"  max_speed  : {best_params['max_speed']}")
    print(f"  min_speed  : {best_params['min_speed']}")
    print(f"""
  ✅ 将最优参数复制到 simulation2.py 的 CONFIG：
  CONFIG = {{
      "waypoints": {WAYPOINTS},
      "lookahead": {best_params['lookahead']},
      "max_speed": {best_params['max_speed']},
      "min_speed": {best_params['min_speed']},
  }}
""")


if __name__ == "__main__":
    main()