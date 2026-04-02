"""
把点位转换成曲线
draw_smooth_path(): 在地上画红线
generate_spline_waypoints():把点位转换成曲线
"""

import pybullet as p
import numpy as np

# ---------- 路径绘制 ----------
def draw_smooth_path(waypoints, color=[1,0,0], line_width=3, num_points=1):
    """绘制平滑曲线(Catmull-Rom)"""
    def catmull_rom_spline(p0, p1, p2, p3, t):
        t2 = t*t
        t3 = t2*t
        return (0.5 * ((2*p1) +
                       (-p0 + p2) * t +
                       (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
                       (-p0 + 3*p1 - 3*p2 + p3) * t3))

    points = np.array(waypoints)
    n = len(points)
    pts = np.vstack([points[0], points, points[-1]])

    for i in range(1, n+1):
        p0 = pts[i-1]
        p1 = pts[i]
        p2 = pts[i+1] if i+1 < len(pts) else pts[i]
        p3 = pts[i+2] if i+2 < len(pts) else pts[i+1]

        prev = None
        for j in range(num_points+1):
            t = j / float(num_points)
            point = catmull_rom_spline(p0, p1, p2, p3, t)
            if prev is not None:
                p.addUserDebugLine([prev[0], prev[1], 0.05],
                                   [point[0], point[1], 0.05],
                                   color, line_width)
            prev = point


def generate_spline_waypoints(waypoints, num_points=200):
    """生成密集的平滑路径点（用于跟随）"""
    def catmull_rom_spline(p0, p1, p2, p3, t):
        t2 = t*t
        t3 = t2*t
        return (0.5 * ((2*p1) +
                       (-p0 + p2) * t +
                       (2*p0 - 5*p1 + 4*p2 - p3) * t2 +
                       (-p0 + 3*p1 - 3*p2 + p3) * t3))

    points = np.array(waypoints)
    n = len(points)
    pts = np.vstack([points[0], points, points[-1]])
    dense = []
    for i in range(1, n+1):
        p0 = pts[i-1]
        p1 = pts[i]
        p2 = pts[i+1] if i+1 < len(pts) else pts[i]
        p3 = pts[i+2] if i+2 < len(pts) else pts[i+1]
        for j in range(num_points):
            t = j / float(num_points)
            point = catmull_rom_spline(p0, p1, p2, p3, t)
            dense.append(point)
    dense.append(waypoints[-1])
    return dense

def Generate_Spline_Path(user_waypoints):
    """
    生成路径点（支持用户输入覆盖 + 可选平滑处理）

    参数:
        user_waypoints: 默认路径点 (list of tuples)
        smooth_follow: 是否进行平滑处理 (bool)

    返回:
        controller_waypoints: 最终用于控制的路径点
    """
    # === 平滑路径处理 ===
    try:
        controller_waypoints = generate_spline_waypoints(user_waypoints)
        print(f"已生成 {len(controller_waypoints)} 个平滑路径点用于跟随。")

    except Exception as e:
        print(f"平滑路径生成失败，使用原始路径点。错误: {e}")
        controller_waypoints = user_waypoints

    return controller_waypoints