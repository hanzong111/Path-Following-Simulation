import numpy as np
import pybullet as p
import math

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

def min_distance_to_path(robot_pos, waypoints):
    """Calculate minimum distance from robot to any path segment."""

    min_dist = float('inf')
    for i in range(len(waypoints) - 1):
        p1 = np.array(waypoints[i])
        p2 = np.array(waypoints[i+1])
        p = np.array(robot_pos[:2])
        seg_vec = p2 - p1
        seg_len_sq = np.dot(seg_vec, seg_vec)
        if seg_len_sq == 0:
            continue
        t = np.dot(p - p1, seg_vec) / seg_len_sq
        t = max(0.0, min(1.0, t))
        closest = p1 + t * seg_vec
        dist = np.linalg.norm(p - closest)
        if dist < min_dist:
            min_dist = dist
    return min_dist