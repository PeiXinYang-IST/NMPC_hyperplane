#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import numpy as np
import math
import random

class TrackGenerator:
    """ 保持与 Path Publisher 一致的赛道生成逻辑 """
    def __init__(self, resolution=0.2):
        self.resolution = resolution
        self.points = [] 
        self.reset()
        
    def reset(self):
        self.points = []
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.s = 0.0
        self.points.append([self.x, self.y, self.yaw, self.s])
        
    def add_straight(self, length):
        steps = int(length / self.resolution)
        for _ in range(steps):
            self.x += self.resolution * np.cos(self.yaw)
            self.y += self.resolution * np.sin(self.yaw)
            self.s += self.resolution
            self.points.append([self.x, self.y, self.yaw, self.s])
            
    def add_turn(self, angle_deg, radius, direction='left'):
        angle_rad = np.radians(angle_deg)
        # 计算弧长
        arc_length = angle_rad * radius
        steps = int(arc_length / self.resolution)
        
        if steps == 0: return
        
        d_yaw = angle_rad / steps
        if direction == 'right': d_yaw = -d_yaw
        
        for _ in range(steps):
            self.yaw += d_yaw
            self.x += self.resolution * np.cos(self.yaw)
            self.y += self.resolution * np.sin(self.yaw)
            self.s += self.resolution
            self.points.append([self.x, self.y, self.yaw, self.s])
            
    def get_path(self): return np.array(self.points)

class SimulationEnv(Node):
    def __init__(self):
        super().__init__('simulation_env')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/scan_cloud', 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # --- Ego 车辆状态 ---
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x, y, yaw, v, omega
        self.current_s = 0.0 # 当前在赛道上的里程 s
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_and_publish)
        
        # --- 生成与 Path Publisher 完全一致的赛道 ---
        self.road_width = 8.0
        self.track_gen = TrackGenerator(resolution=0.2) 
        self.generate_complex_track_data()
        self.track_data = self.track_gen.get_path() # [x, y, yaw, s]
        self.track_length = self.track_data[-1, 3]

        self.get_logger().info(f"Track Generated. Total Length: {self.track_length:.2f}m")

        # 注意：这里不再预生成所有的 static_wall_points，因为赛道太长会导致内存爆炸和计算卡顿
        # 我们将在 update 中使用“滑动窗口”生成局部墙壁

        # --- 复杂障碍物配置 (基于新的赛道几何调整) ---
        # 赛道结构参考:
        # 0-50m: 直道
        # 50-73m: 左转90度 (R15)
        # 73-93m: 直道
        # 93-116m: 右转90度 (R15)
        self.static_obstacles = [
            # 1. 第一个直道末端，入弯前
            {'s': 40.0, 'offset': -2.0, 'width': 1.5, 'length': 1.5},
            
            # 2. 第一个直角弯(左转)的出弯口，逼迫车辆贴内线或减速
            {'s': 70.0, 'offset': 2.0, 'width': 2.0, 'length': 2.0}, 
            
            # 3. 中间短直道上的障碍
            {'s': 85.0, 'offset': 0.0, 'width': 1.5, 'length': 3.0},
            
            # 4. 第二个直角弯(右转)的弯心
            {'s': 105.0, 'offset': -1.5, 'width': 1.5, 'length': 1.5},
            
            # 5. 长直道入口
            {'s': 130.0, 'offset': 1.5, 'width': 2.0, 'length': 5.0},
        ]

        # 动态 NPC
        self.traffic_vehicles = [
            {'id': 1, 's': 20.0, 'speed': 3.0, 'offset': 2.0, 'width': 1.8, 'length': 4.5}, 
            # 这个车会在两个直角弯之间慢慢开
            {'id': 2, 's': 60.0, 'speed': 2.0, 'offset': -2.0,  'width': 1.8, 'length': 4.0}, 
        ]

    def generate_complex_track_data(self):
        """ 必须与 path_publisher.py 保持完全一致 """
        self.track_gen.add_straight(50.0)
        self.track_gen.add_turn(90, 15.0, 'left')   # 直角弯
        self.track_gen.add_straight(20.0)
        self.track_gen.add_turn(90, 15.0, 'right')  # 直角弯
        self.track_gen.add_straight(400.0)
        self.track_gen.add_turn(180, 205.0, 'left') # 大半径回头弯
        self.track_gen.add_turn(45, 200.0, 'right')
        self.track_gen.add_turn(45, 200.0, 'left')
        self.track_gen.add_straight(15000.0)        # 超长直道

    def get_pose_at_s(self, s_req):
        """ 在赛道上插值获取坐标，用于 NPC 和障碍物定位 """
        if s_req >= self.track_length: s_req = s_req % self.track_length
        if s_req < 0: s_req = 0
        
        # 使用 searchsorted 快速查找索引 (track_data[:, 3] 是 s)
        idx = np.searchsorted(self.track_data[:, 3], s_req)
        
        if idx == 0: idx = 1
        if idx >= len(self.track_data): idx = len(self.track_data) - 1
        
        p0 = self.track_data[idx-1]
        p1 = self.track_data[idx]
        
        ratio = (s_req - p0[3]) / (p1[3] - p0[3] + 1e-6)
        x = p0[0] + (p1[0] - p0[0]) * ratio
        y = p0[1] + (p1[1] - p0[1]) * ratio
        
        # 角度插值处理 -pi/pi 问题
        diff = p1[2] - p0[2]
        if diff > np.pi: diff -= 2*np.pi
        if diff < -np.pi: diff += 2*np.pi
        yaw = p0[2] + diff * ratio
        
        return x, y, yaw, idx

    def get_local_wall_points(self, center_idx, look_range_idx=300):
        """ 
        高性能生成墙壁：只生成 center_idx 前后 look_range_idx 范围内的墙壁 
        look_range_idx=300 大约对应 300*0.2 = 60米的范围
        """
        points = []
        half_w = self.road_width / 2.0
        
        start_idx = max(0, center_idx - look_range_idx)
        end_idx = min(len(self.track_data), center_idx + look_range_idx)
        
        # 切片获取局部赛道点
        local_track = self.track_data[start_idx:end_idx:2] # step=2 稍微稀疏一点以提升性能
        
        for pt in local_track:
            x, y, yaw = pt[0], pt[1], pt[2]
            
            cos_yaw_p90 = np.cos(yaw + np.pi/2)
            sin_yaw_p90 = np.sin(yaw + np.pi/2)
            
            # 左墙
            lx = x + half_w * cos_yaw_p90
            ly = y + half_w * sin_yaw_p90
            # 右墙 (利用反向)
            rx = x - half_w * cos_yaw_p90
            ry = y - half_w * sin_yaw_p90
            
            # 生成2层高度: 0m, 1.0m (减少层数提升性能)
            noise_scale = 0.05
            for z in [0.0, 1.0]:
                points.append([lx + np.random.normal(0, noise_scale), ly + np.random.normal(0, noise_scale), z])
                points.append([rx + np.random.normal(0, noise_scale), ry + np.random.normal(0, noise_scale), z])
                
        return points

    def get_box_points(self, cx, cy, w, h, theta):
        """ 生成障碍物点云 """
        pts = []
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        hw, hh = w / 2.0, h / 2.0
        
        # 简化采样点
        x_range = np.linspace(-hw, hw, 3)
        y_range = np.linspace(-hh, hh, 5)
        
        box_local = []
        # 上下边
        for x in x_range:
            box_local.append([x, hh]); box_local.append([x, -hh])
        # 左右边
        for y in y_range:
            box_local.append([hw, y]); box_local.append([-hw, y])
            
        for p in box_local:
            px = cx + p[0] * cos_t - p[1] * sin_t
            py = cy + p[0] * sin_t + p[1] * cos_t
            for z in [0.0, 1.0]: # 2层高度
                pts.append([px + np.random.normal(0, 0.02), py + np.random.normal(0, 0.02), z])
                
        return pts

    def cmd_cb(self, msg):
        self.state[3] = msg.linear.x
        self.state[4] = msg.angular.z

    def update_and_publish(self):
        # 1. Ego 运动积分
        self.state[0] += self.state[3] * np.cos(self.state[2]) * self.dt
        self.state[1] += self.state[3] * np.sin(self.state[2]) * self.dt
        self.state[2] += self.state[4] * self.dt
        
        # 更新当前的 s (粗略估计，用于生成周边环境)
        # 这里为了效率，直接计算离最近路径点的距离更新 s，或者直接利用里程计反推
        # 简单方法：寻找最近的 track point 索引
        dists = (self.track_data[:, 0] - self.state[0])**2 + (self.track_data[:, 1] - self.state[1])**2
        current_idx = np.argmin(dists) # 性能警告：如果 track 太长，全量搜索会慢。
        # 优化：只在上次索引附近搜索。但为了代码稳健性，暂用全量搜索（numpy很快）
        # 对于 75000 个点，argmin 大约耗时 1-2ms，可以接受
        
        self.current_s = self.track_data[current_idx, 3]

        # 2. 动态环境更新
        current_obstacles_points = []
        
        # A. 更新 NPC
        for car in self.traffic_vehicles:
            car['s'] += car['speed'] * self.dt
            if car['s'] > self.track_length: car['s'] = 0.0
            
            # 只处理在视野范围内的 NPC (比如前后 50m)
            dist_s = abs(car['s'] - self.current_s)
            if dist_s < 50.0 or abs(dist_s - self.track_length) < 50.0:
                cx, cy, cyaw, _ = self.get_pose_at_s(car['s'])
                final_x = cx + car['offset'] * np.cos(cyaw + np.pi/2)
                final_y = cy + car['offset'] * np.sin(cyaw + np.pi/2)
                pts = self.get_box_points(final_x, final_y, car['length'], car['width'], cyaw)
                current_obstacles_points.extend(pts)
            
        # B. 生成静态障碍物 (仅附近的)
        for obs in self.static_obstacles:
             dist_s = abs(obs['s'] - self.current_s)
             if dist_s < 50.0:
                cx, cy, cyaw, _ = self.get_pose_at_s(obs['s'])
                final_x = cx + obs['offset'] * np.cos(cyaw + np.pi/2)
                final_y = cy + obs['offset'] * np.sin(cyaw + np.pi/2)
                pts = self.get_box_points(final_x, final_y, obs['length'], obs['width'], cyaw)
                current_obstacles_points.extend(pts)

        # 3. Odometry 发布
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        yaw = self.state[2]
        odom.pose.pose.orientation.z = np.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = np.cos(yaw / 2.0)
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.angular.z = self.state[4]
        self.pub_odom.publish(odom)

        # 4. 感知点云生成 (核心优化部分)
        visible_points = []
        
        # A. 动态生成墙壁点云 (只生成当前索引附近的墙)
        # 视野半径 30m，对应 indices 约 +/- 150 (resolution 0.2)
        local_walls = self.get_local_wall_points(current_idx, look_range_idx=200)
        
        # B. 距离过滤 (模拟激光雷达视距)
        sensing_radius_sq = 30.0 ** 2
        ego_x, ego_y = self.state[0], self.state[1]
        
        # 合并所有点
        all_candidates =  current_obstacles_points
        
        for p in all_candidates:
            # 简单的距离剔除
            if (p[0]-ego_x)**2 + (p[1]-ego_y)**2 < sensing_radius_sq:
                visible_points.append(p)

        if visible_points:
            pc_msg = pc2.create_cloud_xyz32(odom.header, visible_points)
            self.pub_cloud.publish(pc_msg)

def main():
    rclpy.init()
    node = SimulationEnv()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()