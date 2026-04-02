from Compute_Score import compute_score
from PurePursuit import PurePursuit
from drawpath import generate_spline_waypoints
from CarControl import min_distance_to_path
import random
import math
import pybullet as p
import pybullet_data

MAX_ITERATIONS = 50
RESTARTS       = 5

# p.DIRECT，无GUI无sleep
def run_simulation(waypoints, lookahead, max_speed, min_speed):
    controller_waypoints = generate_spline_waypoints(waypoints)

    client   = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)
    robot_id = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.1], physicsClientId=client)

    steering_joints, drive_joints = [], []
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        name = info[1].decode().lower()
        if 'steer' in name:
            steering_joints.append(i)
        elif 'wheel' in name and 'steer' not in name:
            drive_joints.append(i)

    controller = PurePursuit(controller_waypoints, lookahead, max_speed, min_speed)

    trajectory = []   # ✅ added to match GUI version
    errors     = []
    controls   = []
    reached    = False

    for _ in range(5000):   # ✅ same as GUI version
        pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
        yaw      = p.getEulerFromQuaternion(orn)[2]

        v, omega = controller.compute_control(pos[:2], yaw)

        if v == 0:
            steering_angle = 0.0
            wheel_speed    = 0.0
        else:
            steering_angle = max(-0.5, min(0.5, math.atan2(omega * 0.4, v)))
            wheel_speed    = v / 0.1

        for j in steering_joints:
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL,
                                    targetPosition=steering_angle, physicsClientId=client)
        for j in drive_joints:
            p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL,
                                    targetVelocity=wheel_speed, physicsClientId=client)

        p.stepSimulation(physicsClientId=client)

        trajectory.append((pos[0], pos[1]))   # ✅ added
        error = min_distance_to_path(pos, waypoints)
        errors.append(error)
        controls.append((v, omega))

        if math.hypot(pos[0]-waypoints[-1][0], pos[1]-waypoints[-1][1]) < 0.3:
            reached = True
            break

    p.disconnect(client)

    data = {
        "Trajectory": trajectory,   # ✅ same structure as GUI version
        "Error":      errors,
        "controls":   controls,     # ✅ lowercase, matches compute_score
        "Lookahead":  lookahead,
        "Max_Speed":  max_speed,
        "Min_Speed":  min_speed
    }
    score = compute_score(data)
    return score, reached

# ──────────────────────────────────────────
# 工具函数
# ──────────────────────────────────────────
def clamp(value, lo, hi):
    return max(lo, min(hi, value))

def random_params(bounds):
    while True:
        params = {}
        for key, (lo, hi, step) in bounds.items():
            steps       = int((hi - lo) / step)
            params[key] = round(lo + random.randint(0, steps) * step, 4)
        if params["min_speed"] < params["max_speed"]:
            return params


# ──────────────────────────────────────────
# Hill Climbing 核心算法
# ──────────────────────────────────────────
def hill_climbing(waypoints, bounds, start_params, iteration_label=""):
    current          = start_params.copy()
    current_score, _ = run_simulation(waypoints, **current)
    history          = [{"iteration": 0, "params": current.copy(), "score": current_score}]

    print(f"\n  {'─'*45}")
    print(f"  {iteration_label} 起始点: {current} → score={current_score:.4f}")
    print(f"  {'─'*45}")

    for iteration in range(1, MAX_ITERATIONS + 1):
        best_neighbor       = None
        best_neighbor_score = current_score

        for key, (lo, hi, step) in bounds.items():
            for delta in [+step, -step]:
                neighbor      = current.copy()
                neighbor[key] = round(clamp(neighbor[key] + delta, lo, hi), 4)

                if neighbor["min_speed"] >= neighbor["max_speed"]:
                    continue
                if neighbor == current:
                    continue

                score, reached = run_simulation(waypoints, **neighbor)
                status = "✅" if reached else "⏱"
                print(f"    {status} 试 {key}{'+' if delta>0 else ''}{delta}: "
                      f"{neighbor} → score={score:.4f}")

                if score > best_neighbor_score:
                    best_neighbor_score = score
                    best_neighbor       = neighbor.copy()

        if best_neighbor is None:
            print(f"\n  🏔️  第 {iteration} 轮：无更好邻居，已到达局部最优，停止。")
            break

        improvement   = best_neighbor_score - current_score
        current       = best_neighbor
        current_score = best_neighbor_score
        history.append({"iteration": iteration,
                        "params": current.copy(),
                        "score":  current_score})
        print(f"\n  ↗️  第 {iteration} 轮：移动到 {current} "
              f"(score={current_score:.4f}, 提升={improvement:+.4f})\n")

    return current, current_score, history


# ──────────────────────────────────────────
# 随机重启 Hill Climbing
# ──────────────────────────────────────────
def random_restart_hill_climbing(waypoints, bounds, restarts=RESTARTS):
    print("=" * 55)
    print("  🧗 随机重启 Hill Climbing")
    print(f"  重启次数: {restarts}  最大迭代: {MAX_ITERATIONS}")
    print(f"  路径点: {waypoints}")
    print("=" * 55)

    all_results  = []
    best_overall = None
    best_score   = -float('inf')

    for restart in range(restarts):
        start = random_params(bounds)
        label = f"重启 {restart+1}/{restarts}"
        print(f"\n{'='*55}")
        print(f"  🎲 {label} 随机起点: {start}")

        best_params, best_sc, history = hill_climbing(waypoints, bounds, start, label)

        all_results.append({
            "restart":     restart + 1,
            "start":       start,
            "best_params": best_params,
            "best_score":  best_sc,
            "history":     history
        })

        if best_sc > best_score:
            best_score   = best_sc
            best_overall = best_params.copy()
            print(f"\n  🌟 新的全局最优！score={best_score:.4f}")

    return best_overall, best_score, all_results
