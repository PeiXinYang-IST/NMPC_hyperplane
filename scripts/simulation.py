#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import numpy as np

class SimulationEnv(Node):
    def __init__(self):
        super().__init__('simulation_env')
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/scan_cloud', 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        # 机器人状态: [x, y, yaw, v, omega]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_and_publish)
        
        # --- 静态道路 ---
        self.road_width = 10.0  # 路宽稍微加宽一点，方便超车
        self.wall_density = 15
        self.track_length = 200.0
        self.static_road_points = self.generate_road_boundaries()

        # --- 交通流 (NPC 车辆) ---
        # 定义每一辆车的初始状态
        # x: 初始纵向位置
        # speed: 前进速度 (要比 NMPC 的 ref_vel 小)
        # lane_offset: 横向偏移 (+1.5: 左道, -1.5: 右道, 0: 中间)
        self.traffic_vehicles = [
            {'id': 1, 'x': 10.0, 'speed': 1.5, 'offset': -1.5, 'width': 1.8, 'length': 4.0}, # 慢车在右
            {'id': 2, 'x': 25.0, 'speed': 2.0, 'offset': 1.5,  'width': 1.8, 'length': 4.0}, # 慢车在左
            {'id': 3, 'x': 45.0, 'speed': 1.0, 'offset': 0.0,  'width': 2.0, 'length': 5.0}, # 大卡车堵中间
            {'id': 4, 'x': 60.0, 'speed': 2.5, 'offset': -1.2, 'width': 1.8, 'length': 4.0}  # 远处的车
        ]

    def generate_road_boundaries(self):
        """生成静态的正弦波道路边界"""
        points = []
        step = 1.0 / self.wall_density 
        for x in np.arange(0, self.track_length, step):
            # y = 3sin(0.1x)
            y_center = 3.0 * np.sin(0.1 * x)
            dy_dx = 0.3 * np.cos(0.1 * x)
            norm_angle = np.arctan(dy_dx) + np.pi / 2.0
            
            offset = self.road_width / 2.0
            lx = x + offset * np.cos(norm_angle)
            ly = y_center + offset * np.sin(norm_angle)
            rx = x - offset * np.cos(norm_angle)
            ry = y_center - offset * np.sin(norm_angle)
            
            points.append([lx, ly, 0.0])
            points.append([rx, ry, 0.0])
        return points

    def get_vehicle_points(self, cx, cy, w, h, theta):
        """生成车辆的矩形轮廓点云"""
        pts = []
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        hw, hh = w / 2.0, h / 2.0
        
        # 矩形周长采样
        density = 8 # 点密度
        # 前后边
        for y in np.linspace(-hh, hh, int(h*density)):
            pts.append([hw, y])  # 前保险杠
            pts.append([-hw, y]) # 后保险杠
        # 左右边
        for x in np.linspace(-hw, hw, int(w*density)):
            pts.append([x, hh])  # 左车门
            pts.append([x, -hh]) # 右车门
            
        # 转换到世界坐标
        world_pts = []
        for p in pts:
            px = cx + p[0] * cos_t - p[1] * sin_t
            py = cy + p[0] * sin_t + p[1] * cos_t
            # 加上一点随机噪声，模拟激光雷达扫在车身上的不确定性
            px += np.random.normal(0, 0.03)
            py += np.random.normal(0, 0.03)
            world_pts.append([px, py, 0.0])
        return world_pts

    def cmd_cb(self, msg):
        self.state[3] = msg.linear.x
        self.state[4] = msg.angular.z

    def update_and_publish(self):
        # 1. 自身运动学积分 (Ego Vehicle)
        self.state[0] += self.state[3] * np.cos(self.state[2]) * self.dt
        self.state[1] += self.state[3] * np.sin(self.state[2]) * self.dt
        self.state[2] += self.state[4] * self.dt

        # 2. 交通流更新 (Traffic Simulation)
        traffic_points = []
        for car in self.traffic_vehicles:
            # A. 更新位置 (沿 X 轴简单推算，假设沿路线走)
            # 为了更精确，应该沿曲线弧长更新，但近似计算 X 增量足够用于仿真
            # dx = v * dt * cos(theta)
            # 先算当前角度
            heading = np.arctan(0.3 * np.cos(0.1 * car['x']))
            car['x'] += car['speed'] * self.dt * np.cos(heading)
            
            # B. 计算世界坐标
            y_path = 3.0 * np.sin(0.1 * car['x'])
            norm_angle = heading + np.pi / 2.0
            
            # 加上横向偏移 (Lane Offset)
            final_x = car['x'] + car['offset'] * np.cos(norm_angle)
            final_y = y_path + car['offset'] * np.sin(norm_angle)
            
            # C. 生成车辆点云
            car_pts = self.get_vehicle_points(final_x, final_y, car['length'], car['width'], heading)
            traffic_points.extend(car_pts)

        # 3. 发布里程计
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

        # 4. 合并点云 (道路 + 车辆)
        # 性能优化：只提取机器人附近的道路点
        visible_points = []
        
        # A. 附近的道路
        # for p in self.static_road_points:
        #     if (p[0] - self.state[0])**2 + (p[1] - self.state[1])**2 < 625.0: # 25m 半径
        #         nx = p[0] + np.random.normal(0, 0.02)
        #         ny = p[1] + np.random.normal(0, 0.02)
        #         visible_points.append([nx, ny, 0.0])
        
        # B. 附近的车辆 (全加进去，因为车辆少)
        visible_points.extend(traffic_points)

        pc_msg = pc2.create_cloud_xyz32(odom.header, visible_points)
        self.pub_cloud.publish(pc_msg)

def main():
    rclpy.init()
    rclpy.spin(SimulationEnv())
    rclpy.shutdown()

if __name__ == '__main__':
    main()