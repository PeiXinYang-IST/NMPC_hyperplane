#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import numpy as np
import math

class SimulationEnv(Node):
    def __init__(self):
        super().__init__('simulation_env')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/scan_cloud', 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # --- Ego 车辆状态 [x, y, yaw, v, omega] ---
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_and_publish)
        
        # --- 静态道路参数 ---
        self.road_width = 8.0   # 稍微变窄，增加难度
        self.wall_density = 10  # 边界点密度
        self.track_length = 300.0
        # 预计算所有道路边界点
        self.static_road_points = self.generate_road_boundaries()

        # --- 复杂场景设置 ---
        # 1. 静态障碍物 (模拟施工路障/抛锚车)
        self.static_obstacles = [
            {'x': 40.0, 'offset': 0.0, 'width': 2.0, 'length': 2.0},  # 路中间的路障，逼迫绕行
            {'x': 90.0, 'offset': 2.0, 'width': 2.0, 'length': 4.0},  # 左侧车道堵死
            {'x': 150.0, 'offset':-2.0, 'width': 2.0, 'length': 3.0}, # 右侧车道堵死
        ]

        # 2. 动态交通流 (NPC)
        self.traffic_vehicles = [
            # ID 1: 极慢的右侧车 (逼迫超车)
            {'id': 1, 'x': 15.0, 'speed': 1.0, 'offset': -2.0, 'width': 1.8, 'length': 4.5}, 
            # ID 2: 中速左侧车 (伴行干扰)
            {'id': 2, 'x': 20.0, 'speed': 3.5, 'offset': 2.0,  'width': 1.8, 'length': 4.0}, 
            # ID 3: 远处的快速车
            {'id': 3, 'x': 60.0, 'speed': 4.0, 'offset': -1.5, 'width': 2.0, 'length': 5.0}, 
            # ID 4: 这是一个陷阱，两车并排 (x=110左右)，看NMPC是否减速
            {'id': 4, 'x': 110.0, 'speed': 0.5, 'offset': -2.0, 'width': 2.0, 'length': 4.0},
            {'id': 5, 'x': 110.0, 'speed': 0.5, 'offset': 2.0,  'width': 2.0, 'length': 4.0},
        ]

    def get_road_y(self, x):
        """ 复杂的复合道路方程 """
        # 复合波：主波 + 低频波，模拟不规则弯道
        return 4.0 * np.sin(0.1 * x) + 2.0 * np.cos(0.05 * x)

    def get_road_dy_dx(self, x):
        """ 道路切线斜率 (用于计算朝向) """
        return 0.4 * np.cos(0.1 * x) - 0.1 * np.sin(0.05 * x)

    def generate_road_boundaries(self):
        """生成静态道路边界点云"""
        points = []
        step = 1.0 / self.wall_density 
        for x in np.arange(0, self.track_length, step):
            y_center = self.get_road_y(x)
            dy_dx = self.get_road_dy_dx(x)
            norm_angle = np.arctan(dy_dx) + np.pi / 2.0
            
            # offset = self.road_width / 2.0
            # # 左边界
            # lx = x + offset * np.cos(norm_angle)
            # ly = y_center + offset * np.sin(norm_angle)
            # # 右边界
            # rx = x - offset * np.cos(norm_angle)
            # ry = y_center - offset * np.sin(norm_angle)
            
            # points.append([lx, ly, 0.0])
            # points.append([rx, ry, 0.0])
        return points

    def get_box_points(self, cx, cy, w, h, theta):
        """生成矩形(车辆/障碍物)的轮廓点云"""
        pts = []
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        hw, hh = w / 2.0, h / 2.0
        
        # 稀疏采样即可，不用太密集
        density_x = int(w * 4)
        density_y = int(h * 4)
        
        local_pts = []
        # 四条边
        for x in np.linspace(-hw, hw, density_x):
            local_pts.append([x, hh])
            local_pts.append([x, -hh])
        for y in np.linspace(-hh, hh, density_y):
            local_pts.append([hw, y])
            local_pts.append([-hw, y])
            
        # 转换到世界坐标
        world_pts = []
        for p in local_pts:
            px = cx + p[0] * cos_t - p[1] * sin_t
            py = cy + p[0] * sin_t + p[1] * cos_t
            # 加入少量传感器噪声
            px += np.random.normal(0, 0.02)
            py += np.random.normal(0, 0.02)
            world_pts.append([px, py, 0.0])
        return world_pts

    def cmd_cb(self, msg):
        # 简单的执行器响应模型
        self.state[3] = msg.linear.x
        self.state[4] = msg.angular.z

    def update_and_publish(self):
        # 1. Ego Vehicle 运动学积分 (Simple Bicycle Model)
        self.state[0] += self.state[3] * np.cos(self.state[2]) * self.dt
        self.state[1] += self.state[3] * np.sin(self.state[2]) * self.dt
        self.state[2] += self.state[4] * self.dt

        # 2. 动态环境更新
        current_obstacles_points = []
        
        # A. 更新 NPC 车辆
        for car in self.traffic_vehicles:
            # 获取当前位置的道路切向角
            road_dy = self.get_road_dy_dx(car['x'])
            path_heading = np.arctan(road_dy)
            
            # 更新纵向位置 (假设沿着切线方向行驶)
            car['x'] += car['speed'] * self.dt * np.cos(path_heading) # 简化的纵向投影
            
            # 计算世界坐标
            y_center = self.get_road_y(car['x'])
            norm_angle = path_heading + np.pi / 2.0
            
            final_x = car['x'] + car['offset'] * np.cos(norm_angle)
            final_y = y_center + car['offset'] * np.sin(norm_angle)
            
            # 生成点云
            pts = self.get_box_points(final_x, final_y, car['length'], car['width'], path_heading)
            current_obstacles_points.extend(pts)

        # B. 生成静态障碍物点云
        for obs in self.static_obstacles:
            road_dy = self.get_road_dy_dx(obs['x'])
            obs_heading = np.arctan(road_dy) # 障碍物通常摆正
            
            y_center = self.get_road_y(obs['x'])
            norm_angle = obs_heading + np.pi / 2.0
            final_x = obs['x'] + obs['offset'] * np.cos(norm_angle)
            final_y = y_center + obs['offset'] * np.sin(norm_angle)
            
            pts = self.get_box_points(final_x, final_y, obs['length'], obs['width'], obs_heading)
            current_obstacles_points.extend(pts)

        # 3. 发布 Odometry
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

        # 4. 发布感知点云 (Sensor Fusion)
        # 优化：仅提取 Ego 车身周围 R 米内的点，模拟真实雷达范围，减轻 Rviz/NMPC 负担
        sensing_radius = 40.0 
        visible_points = []
        
        # A. 筛选道路边界 (KD-Tree会更快，但这里简单循环优化一下)
        # 我们只遍历索引在当前 x 附近的点
        ego_x = self.state[0]
        # 粗略估算索引范围 (dx ~ 0.1)
        start_idx = max(0, int((ego_x - sensing_radius) * self.wall_density * 2)) 
        end_idx = min(len(self.static_road_points), int((ego_x + sensing_radius) * self.wall_density * 2))
        
        for i in range(start_idx, end_idx):
            p = self.static_road_points[i]
            dist_sq = (p[0] - self.state[0])**2 + (p[1] - self.state[1])**2
            if dist_sq < sensing_radius**2:
                visible_points.append(p)
        
        # B. 加入所有障碍物 (假设障碍物不多，不做剔除)
        # 实际应该也做距离剔除
        for p in current_obstacles_points:
             dist_sq = (p[0] - self.state[0])**2 + (p[1] - self.state[1])**2
             if dist_sq < sensing_radius**2:
                visible_points.append(p)

        if visible_points:
            pc_msg = pc2.create_cloud_xyz32(odom.header, visible_points)
            self.pub_cloud.publish(pc_msg)

def main():
    rclpy.init()
    node = SimulationEnv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()