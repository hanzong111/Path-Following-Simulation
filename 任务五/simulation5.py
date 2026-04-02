"""
任务 2:路径跟踪控制（自定义路径点 + 可调参数 + 平滑路径可选）

输入：自定义二维路径点
输出：线速度 v, 角速度 ω
验收标准：机器人沿路径移动，跟踪误差收敛
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from drawpath import draw_smooth_path, generate_spline_waypoints
from PurePursuit import PurePursuit
from Compute_Score import compute_score
from Save_Json import save_simulation_result
from drawpath import Generate_Spline_Path
from CarControl import apply_car_control , min_distance_to_path



# =============================================
# ✅ 在这里直接修改你的路径点和参数！
# =============================================
CONFIG = {
    "waypoints": [(0.0, 0.0), (4, 1), (8, -1)],  # ← 改这里
    "lookahead": 2,
    "max_speed": 1,
    "min_speed": 0.6,
}
# =============================================

# ---------- 仿真初始化与汽车控制 ----------
def initialize():
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.1])
    p.resetDebugVisualizerCamera(cameraDistance=10.0,
                                                 cameraYaw=90,
                                                 cameraPitch=-40,
                                                 cameraTargetPosition=[0,0,0])
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # ✅ 关闭 Explorer / Test / Params 面板

    steering_joints = []
    drive_joints = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode('utf-8').lower()
        if 'steer' in name:
            steering_joints.append(i)
        elif 'wheel' in name and 'steer' not in name:
            drive_joints.append(i)

    print(f"转向关节索引: {steering_joints}")
    print(f"驱动关节索引: {drive_joints}")
    return robot_id, steering_joints, drive_joints
    
# ---------- 主程序 ----------
def main():
    user_waypoints = CONFIG["waypoints"]
    lookahead      = CONFIG["lookahead"]
    max_speed      = CONFIG["max_speed"]
    min_speed      = CONFIG["min_speed"]

    controller_waypoints = Generate_Spline_Path(user_waypoints)
    robot_id, steering_joints, drive_joints = initialize()
    draw_smooth_path(controller_waypoints, color=[1, 0, 0], line_width=3)
    controller = PurePursuit(controller_waypoints, lookahead, max_speed, min_speed)

    dt        = 1.0 / 240.0
    max_steps = 10000

    trajectory = []
    errors     = []
    controls   = []
    reached    = False   # ← moved outside try so finally can see it

    print("\n开始路径跟踪...")

    try:
        for step in range(max_steps):
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            yaw      = p.getEulerFromQuaternion(orn)[2]

            v, omega = controller.compute_control(pos[:2], yaw)
            apply_car_control(robot_id, steering_joints, drive_joints, v, omega)

            cam_yaw = math.degrees(yaw - math.pi / 2) % 360
            p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=cam_yaw,
                                         cameraPitch=-40, cameraTargetPosition=pos)
            p.stepSimulation()
            time.sleep(dt)

            trajectory.append((pos[0], pos[1]))
            error = min_distance_to_path(pos, user_waypoints)
            errors.append(error)
            controls.append((v, omega))

            if step % (int(0.5 / dt)) == 0:
                print(f"步 {step}: 位置=({pos[0]:.2f},{pos[1]:.2f}), v={v:.2f}, ω={omega:.2f}, 误差={error:.2f}")

            if math.hypot(pos[0]-user_waypoints[-1][0], pos[1]-user_waypoints[-1][1]) < 0.3:
                print("已到达终点！")
                reached = True
                break

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        save_simulation_result(trajectory, errors, controls, lookahead, max_speed, min_speed, folder="data")
        data = {
            "Trajectory": trajectory,
            "Error":      errors,
            "controls":   controls,   # ✅ 改成小写，与 compute_score 一致
            "Lookahead":  lookahead,
            "Max_Speed":  max_speed,
            "Min_Speed":  min_speed
        }
if __name__ == "__main__":
    main()