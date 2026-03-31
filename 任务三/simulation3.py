"""
任务 2:路径跟踪控制（自定义路径点 + 可调参数 + 平滑路径可选）

输入：自定义二维路径点
输出：线速度 v, 角速度 ω
验收标准：机器人沿路径移动，跟踪误差收敛
"""

import pybullet as p
import pybullet_data
import time
import math
import json
import os
from drawpath import draw_smooth_path, generate_spline_waypoints
from PurePursuit import PurePursuit

# =============================================
# ✅ 在这里直接修改你的路径点和参数！
# =============================================
CONFIG = {
    "waypoints": [(0.0, 0.0), (5.0, 0.0), (5, 10), (3, 8)],  # ← 改这里
    "lookahead": 0.5,
    "max_speed": 2.5,
    "min_speed": 0.5,
    "smooth_follow": True,
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
    return robot_id, plane_id, steering_joints, drive_joints

def apply_car_control(robot_id, steering_joints, drive_joints, v, omega):
    if v == 0:
        steering_angle = 0.0
        wheel_speed = 0.0
    else:
        wheelbase = 0.4
        steering_angle = math.atan2(omega * wheelbase, v)
        steering_angle = max(-0.5, min(0.5, steering_angle))
        wheel_radius = 0.1
        wheel_speed = v / wheel_radius

    for joint in steering_joints:
        p.setJointMotorControl2(robot_id, joint, p.POSITION_CONTROL,
                                targetPosition=steering_angle)
    for joint in drive_joints:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=wheel_speed)

# ---------- 主程序 ----------
def main():
    print("=== 路径跟踪参数设置 ===")

    user_waypoints = CONFIG["waypoints"]
    lookahead      = CONFIG["lookahead"]
    max_speed      = CONFIG["max_speed"]
    min_speed      = CONFIG["min_speed"]
    smooth_follow  = CONFIG["smooth_follow"]

    try:
        wp_input = input(f"路径点 (回车使用 CONFIG 默认 {user_waypoints}) ").strip()
        if wp_input:
            wp_list = wp_input.split(';')
            user_waypoints = []
            for wp in wp_list:
                x, y = map(float, wp.split(','))
                user_waypoints.append((x, y))
            print(f"✅ 已覆盖 CONFIG,使用输入路径点: {user_waypoints}")
        else:
            print(f"✅ 使用 CONFIG 路径点: {user_waypoints}")
    except EOFError:
        print(f"⚠️ 无法读取 input()，使用 CONFIG 路径点: {user_waypoints}")

    if smooth_follow:
        try:
            controller_waypoints = generate_spline_waypoints(user_waypoints)
            print(f"已生成 {len(controller_waypoints)} 个平滑路径点用于跟随。")
        except Exception as e:
            print(f"平滑路径生成失败，使用原始路径点。错误: {e}")
            controller_waypoints = user_waypoints
            smooth_follow = False
    else:
        controller_waypoints = user_waypoints

    robot_id, plane_id, steering_joints, drive_joints = initialize()
    draw_smooth_path(controller_waypoints, color=[1, 0, 0], line_width=3)
    controller = PurePursuit(controller_waypoints, lookahead, max_speed, min_speed)

    dt = 1.0 / 240.0
    max_steps = 10000
    follow_cam = True

    trajectory = []
    errors = []
    controls = []

    print("\n开始路径跟踪...")
    print(f"原始路径点: {user_waypoints}")
    print(f"控制器参数: 前视距离={lookahead}, 最大速度={max_speed}, 最小速度={min_speed}")
    print("跟随摄像头已启用\n")

    try:
        for step in range(max_steps):
            pos, orn = p.getBasePositionAndOrientation(robot_id)
            euler = p.getEulerFromQuaternion(orn)
            yaw = euler[2]

            v, omega = controller.compute_control(pos[:2], yaw)
            apply_car_control(robot_id, steering_joints, drive_joints, v, omega)

            if follow_cam:
                try:
                    cam_yaw = yaw - math.pi / 2
                    cam_yaw = cam_yaw % (2 * math.pi)
                    cam_yaw_deg = math.degrees(cam_yaw)
                    p.resetDebugVisualizerCamera(cameraDistance=2.0,
                                                 cameraYaw=cam_yaw_deg,
                                                 cameraPitch=-40,
                                                 cameraTargetPosition=pos)
                except:
                    pass

            p.stepSimulation()
            time.sleep(dt)

            trajectory.append((pos[0], pos[1]))
            dists = [math.hypot(pos[0] - wp[0], pos[1] - wp[1]) for wp in user_waypoints]
            error = min(dists)
            errors.append(error)
            controls.append((v, omega))

            if step % (int(0.5 / dt)) == 0:
                print(f"步 {step}: 位置=({pos[0]:.2f},{pos[1]:.2f}), v={v:.2f}, ω={omega:.2f}, 误差={error:.2f}")

            if math.hypot(pos[0] - user_waypoints[-1][0], pos[1] - user_waypoints[-1][1]) < 0.3:
                print("已到达终点！")
                break

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        os.makedirs("data", exist_ok=True)
        data = {
            "Trajectory": trajectory,
            "Error": errors,
            "Cotrol": controls,  # (v, omega) per step
        }
        with open("data/task_result.json", "w") as f:
            json.dump(data, f, indent=2)
        print("数据已保存到 data/task_result.json")
        time.sleep(0.5)
        p.disconnect()
        print("仿真结束")

if __name__ == "__main__":
    main()