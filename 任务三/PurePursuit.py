"""
纯跟踪控制器 

"""
import math

# ---------- 纯跟踪控制器 ----------
class PurePursuit:
    def __init__(self, waypoints, lookahead_distance=1.5, max_speed=2.0, min_speed=0.5):
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.idx = 0

    def find_target_point(self, position):
        dists = [math.hypot(position[0]-wp[0], position[1]-wp[1]) for wp in self.waypoints]
        self.idx = dists.index(min(dists))

        cumulative = 0.0
        for i in range(self.idx, len(self.waypoints)-1):
            p1 = self.waypoints[i]
            p2 = self.waypoints[i+1]
            seg_len = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
            if cumulative + seg_len >= self.lookahead_distance:
                ratio = (self.lookahead_distance - cumulative) / seg_len
                target = (p1[0] + ratio*(p2[0]-p1[0]),
                          p1[1] + ratio*(p2[1]-p1[1]))
                return target
            cumulative += seg_len
        return self.waypoints[-1]

    def compute_control(self, robot_pos, robot_yaw):
        target = self.find_target_point(robot_pos)
        dx = target[0] - robot_pos[0]
        dy = target[1] - robot_pos[1]
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - robot_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        k_steering = 1.5
        omega = k_steering * angle_diff
        omega = max(-1.0, min(1.0, omega))

        v = self.max_speed * (1.0 - 0.5 * abs(angle_diff))
        v = max(self.min_speed, min(self.max_speed, v))

        return v, omega
