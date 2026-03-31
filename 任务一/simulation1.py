"""
任务 1:仿真环境搭建 - 真实汽车控制

功能概述：
    1. 加载 PyBullet 仿真环境（地面 + 四轮赛车）。
    2. 实现真实汽车控制：根据用户输入的线速度 v 和角速度 ω，
       使用自行车模型计算前轮转向角，并将 v 转换为后轮驱动速度。
    3. 实现两段运动：前进 5 秒，转弯 5 秒（用户可自定义 v 和 ω）。

控制原理：
    - 真实汽车运动学模型（自行车模型）：
        转向角 δ = atan(ω * L / v)    (L 为轴距，单位：米）
        驱动轮转速 ω_wheel = v / r    (r 为车轮半径，单位：米）
    - 通过 `setJointMotorControl2` 设置前轮转向关节的转角（位置控制），
      后轮驱动关节的目标速度（速度控制）。

关节识别：
    - 遍历机器人所有关节，根据名称包含 "steer" 识别转向关节，
      包含 "wheel" 但不含 "steer" 识别为驱动轮关节。
    - 这样即使 URDF 模型有微小变化，代码也能自动适应。
"""

"""
任务 1:仿真环境搭建 - 真实汽车控制 + 跟随摄像头
"""

import pybullet as p
import pybullet_data
import time
import math

def initialize():
    # 初始化仿真环境
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("racecar/racecar.urdf", [0, 0, 0.1])
    p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=90, cameraPitch=-20, cameraTargetPosition=[0, 0, 0])

    # 获取关节索引
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
    """
    真实汽车控制：根据 v 和 ω 计算转向角和驱动速度
    """
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

def main():
    print("=== 机器人运动控制（真实汽车）===")
    print("============= 前进 ============")
    v_forward = float(input("前进线速度 v (默认 2.0): ") or "2.0")
    omega_forward = 0
    print("============= 转弯 ============")
    v_turn = float(input("转弯线速度 v (默认 1.0): ") or "1.0")
    omega_turn = float(input("转弯角速度 ω (默认 1.0 rad/s): ") or "1.0")
    follow_cam = 'y'

    robot_id, plane_id, steering_joints, drive_joints = initialize()

    dt = 1.0 / 240.0
    phase_duration = 5.0
    phase_steps = int(phase_duration / dt)

    print(f"\n开始仿真。第一阶段：前进 {phase_duration} 秒 (v={v_forward}, ω={omega_forward})")
    print(f"第二阶段：转弯 {phase_duration} 秒 (v={v_turn}, ω={omega_turn})")
    print("控制模式: 真实汽车（前轮转向 + 后轮驱动）")
    if follow_cam:
        print("跟随摄像头: 已开启 (视角位于车左侧 90° 方向)")
    else:
        print("跟随摄像头: 已关闭")
    print()

    try:
        # 第一阶段：前进
        for step in range(phase_steps):
            apply_car_control(robot_id, steering_joints, drive_joints, v_forward, omega_forward)
            p.stepSimulation()

            # 跟随摄像头：每帧更新
            if follow_cam:
                try:
                    pos, orn = p.getBasePositionAndOrientation(robot_id)
                    euler = p.getEulerFromQuaternion(orn)
                    yaw = euler[2]
                    # 相机偏航角 = 车头方向 - 90° (顺时针)
                    cam_yaw = yaw - math.pi/2
                    cam_yaw = cam_yaw % (2 * math.pi)   # 归一化
                    cam_yaw_deg = math.degrees(cam_yaw)
                    p.resetDebugVisualizerCamera(cameraDistance=2.0,
                                                cameraYaw=cam_yaw_deg,
                                                cameraPitch=-40,
                                                cameraTargetPosition=pos)
                except:
                    pass

            time.sleep(dt)

            if step % (int(0.5 / dt)) == 0:
                pos, _ = p.getBasePositionAndOrientation(robot_id)
                print(f"[前进] 步 {step}: 位置 = ({pos[0]:.2f}, {pos[1]:.2f})")

        # 第二阶段：转弯
        for step in range(phase_steps):
            apply_car_control(robot_id, steering_joints, drive_joints, v_turn, omega_turn)
            p.stepSimulation()

            if follow_cam:
                try:
                    pos, orn = p.getBasePositionAndOrientation(robot_id)
                    euler = p.getEulerFromQuaternion(orn)
                    yaw = euler[2]
                    cam_yaw = yaw - math.pi/2
                    cam_yaw = cam_yaw % (2 * math.pi)
                    cam_yaw_deg = math.degrees(cam_yaw)
                    p.resetDebugVisualizerCamera(cameraDistance=2.0,
                                                cameraYaw=cam_yaw_deg,
                                                cameraPitch=-40,
                                                cameraTargetPosition=pos)
                except:
                    pass

            time.sleep(dt)

            if step % (int(0.5 / dt)) == 0:
                pos, _ = p.getBasePositionAndOrientation(robot_id)
                print(f"[转弯] 步 {step}: 位置 = ({pos[0]:.2f}, {pos[1]:.2f})")

    except KeyboardInterrupt:
        print("\n用户中断仿真。")

    finally:
        time.sleep(0.5)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.disconnect()
        print("仿真结束，已断开连接。")

if __name__ == "__main__":
    main()