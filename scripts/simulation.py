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
    """ 复制一份 TrackGenerator 以保持逻辑一致 (实际工程中应放入共享库) """
    def __init__(self, resolution=0.2):
        self.resolution = resolution
        self.points = [] 
        self.reset()
    def reset(self):
        self.points = []; self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.s = 0.0
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
        steps = int((angle_rad * radius) / self.resolution)
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
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_and_publish)
        
        # --- 生成赛道数据 ---
        self.road_width = 8.0
        self.track_gen = TrackGenerator(resolution=0.2) # 分辨率越高，碰撞检测越准
        self.generate_complex_track_data()
        self.track_data = self.track_gen.get_path() # [x, y, yaw, s]
        self.track_length = self.track_data[-1, 3]

        # 预生成静态墙壁点云 (带高度和噪声)
        self.static_wall_points = self.generate_3d_walls()

        # --- 复杂障碍物配置 (基于里程 s) ---
        # 相比基于 x 坐标，基于 s 坐标可以适应任意弯道
        self.static_obstacles = [
            # 直道上的路障
            {'s': 40.0, 'offset': 0.0, 'width': 2.0, 'length': 2.0},
            # 入弯前的陷阱 (左侧堵死)
            {'s': 90.0, 'offset': 2.0, 'width': 2.0, 'length': 4.0}, 
            # 弯心处的障碍 (逼迫走外线)
            {'s': 140.0, 'offset': -2.0, 'width': 1.5, 'length': 1.5},
            # 回头弯后的障碍
            {'s': 220.0, 'offset': 1.0, 'width': 2.0, 'length': 3.0},
        ]

        # 动态 NPC (基于 s 运动)
        self.traffic_vehicles = [
            {'id': 1, 's': 15.0, 'speed': 2.0, 'offset': -2.5, 'width': 1.8, 'length': 4.5}, 
            {'id': 2, 's': 60.0, 'speed': 4.5, 'offset': 2.0,  'width': 1.8, 'length': 4.0}, 
            # 这里的车会通过直角弯
            {'id': 3, 's': 110.0, 'speed': 3.0, 'offset': -1.5, 'width': 2.0, 'length': 5.0}, 
        ]

    def generate_complex_track_data(self):
        """ 必须与 path_publisher 保持完全一致 """
        self.track_gen.add_straight(50.0)
        self.track_gen.add_turn(90, 15.0, 'left')
        self.track_gen.add_straight(20.0)
        self.track_gen.add_turn(90, 15.0, 'right')
        self.track_gen.add_straight(40.0)
        self.track_gen.add_turn(180, 25.0, 'left')
        self.track_gen.add_turn(45, 20.0, 'right')
        self.track_gen.add_turn(45, 20.0, 'left')
        self.track_gen.add_straight(50.0)

    def get_pose_at_s(self, s_req):
        """ 在赛道上插值获取坐标，用于 NPC 和障碍物定位 """
        # 简单查找最近点 (可以优化为二分查找)
        # 这里假设 track_data 的 s 是递增的
        if s_req >= self.track_length:
            s_req = self.track_length - 0.1
        
        # 找到 idx 使得 track[idx].s <= s_req < track[idx+1].s
        # 简单的线性搜索优化：因为车辆通常只向前走，这里直接暴力搜或二分
        # 为了代码简单，用 numpy searchsorted
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
        
        return x, y, yaw

    def generate_3d_walls(self):
        """ 生成带高度的墙壁点云 """
        points = []
        half_w = self.road_width / 2.0
        
        for i in range(0, len(self.track_data), 2): # 稍微稀疏一点
            pt = self.track_data[i]
            x, y, yaw = pt[0], pt[1], pt[2]
            
            # 左右墙基点
            lx = x + half_w * np.cos(yaw + np.pi/2)
            ly = y + half_w * np.sin(yaw + np.pi/2)
            rx = x + half_w * np.cos(yaw - np.pi/2)
            ry = y + half_w * np.sin(yaw - np.pi/2)
            
            # 生成3层高度: 0m, 0.5m, 1.0m
            for z in [0.0, 0.5, 1.0]:
                # 加入随机噪声
                noise = np.random.normal(0, 0.05, 2)
                points.append([lx + noise[0], ly + noise[1], z])
                points.append([rx + noise[0], ry + noise[1], z])
                
        return points

    def get_box_points(self, cx, cy, w, h, theta):
        """ 生成障碍物/车辆点云 (矩形) """
        pts = []
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        hw, hh = w / 2.0, h / 2.0
        
        # 表面采样
        for x in np.linspace(-hw, hw, 5):
            pts.append([x, hh])
            pts.append([x, -hh])
        for y in np.linspace(-hh, hh, 8):
            pts.append([hw, y])
            pts.append([-hw, y])
            
        world_pts = []
        for p in pts:
            # 旋转 + 平移
            px = cx + p[0] * cos_t - p[1] * sin_t
            py = cy + p[0] * sin_t + p[1] * cos_t
            
            # 简单的 3D 挤出 (0~1.5m)
            for z in np.linspace(0, 1.5, 3):
                # 传感器噪声
                n_x = np.random.normal(0, 0.03)
                n_y = np.random.normal(0, 0.03)
                n_z = np.random.normal(0, 0.03)
                world_pts.append([px + n_x, py + n_y, z + n_z])
                
        return world_pts

    def cmd_cb(self, msg):
        self.state[3] = msg.linear.x 
        self.state[4] = msg.angular.z

    def update_and_publish(self):
        # 1. Ego 运动积分
        self.state[0] += self.state[3] * np.cos(self.state[2]) * self.dt
        self.state[1] += self.state[3] * np.sin(self.state[2]) * self.dt
        self.state[2] += self.state[4] * self.dt

        # 2. 动态环境更新
        current_obstacles_points = []
        
        # 更新 NPC
        for car in self.traffic_vehicles:
            # 沿着 s 运动
            car['s'] += car['speed'] * self.dt
            if car['s'] > self.track_length:
                car['s'] = 0.0 # 循环跑
                
            # 计算世界坐标
            cx, cy, cyaw = self.get_pose_at_s(car['s'])
            
            # 应用横向偏移 (offset)
            final_x = cx + car['offset'] * np.cos(cyaw + np.pi/2)
            final_y = cy + car['offset'] * np.sin(cyaw + np.pi/2)
            
            # 生成点云
            pts = self.get_box_points(final_x, final_y, car['length'], car['width'], cyaw)
            current_obstacles_points.extend(pts)
            
        # 生成静态障碍物
        for obs in self.static_obstacles:
            cx, cy, cyaw = self.get_pose_at_s(obs['s'])
            final_x = cx + obs['offset'] * np.cos(cyaw + np.pi/2)
            final_y = cy + obs['offset'] * np.sin(cyaw + np.pi/2)
            pts = self.get_box_points(final_x, final_y, obs['length'], obs['width'], cyaw)
            current_obstacles_points.extend(pts)

        # 3. Odometry
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

        # 4. 感知点云生成 (Sensor Simulation)
        visible_points = []
        sensing_radius = 30.0 # 雷达半径
        ego_x, ego_y = self.state[0], self.state[1]
        
        # A. 筛选墙壁 (简单距离筛选)
        # 优化：只检查最近的一批点太麻烦，直接暴力距离判断 (对于 3000 个点还可以接受)
        # for p in self.static_wall_points:
        #     if abs(p[0] - ego_x) < sensing_radius and abs(p[1] - ego_y) < sensing_radius:
        #         if (p[0]-ego_x)**2 + (p[1]-ego_y)**2 < sensing_radius**2:
        #             visible_points.append(p)
        
        # B. 障碍物点云
        for p in current_obstacles_points:
            if (p[0]-ego_x)**2 + (p[1]-ego_y)**2 < sensing_radius**2:
                visible_points.append(p)

        # C. 添加一些环境杂波 (Outliers)
        # for _ in range(10):
        #     rx = ego_x + random.uniform(-10, 10)
        #     ry = ego_y + random.uniform(-10, 10)
        #     visible_points.append([rx, ry, random.uniform(0, 0.5)])

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