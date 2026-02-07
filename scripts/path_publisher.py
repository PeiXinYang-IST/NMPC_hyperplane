#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class TrackGenerator:
    """ 赛道生成器：支持直线、圆弧拼接 """
    def __init__(self, resolution=0.2):
        self.resolution = resolution
        # 存储航点 [x, y, yaw, cumulative_dist]
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
        """ 添加直线段 """
        steps = int(length / self.resolution)
        for _ in range(steps):
            self.x += self.resolution * np.cos(self.yaw)
            self.y += self.resolution * np.sin(self.yaw)
            self.s += self.resolution
            self.points.append([self.x, self.y, self.yaw, self.s])

    def add_turn(self, angle_deg, radius, direction='left'):
        """ 添加圆弧弯道 """
        angle_rad = np.radians(angle_deg)
        arc_length = angle_rad * radius
        steps = int(arc_length / self.resolution)
        if steps == 0: return

        # 每一步的角度变化量
        d_yaw = angle_rad / steps
        if direction == 'right':
            d_yaw = -d_yaw

        for _ in range(steps):
            self.yaw += d_yaw
            self.x += self.resolution * np.cos(self.yaw)
            self.y += self.resolution * np.sin(self.yaw)
            self.s += self.resolution
            self.points.append([self.x, self.y, self.yaw, self.s])

    def get_path(self):
        return np.array(self.points)

class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')
        self.pub_path = self.create_publisher(Path, '/ref_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path) 
        
        # --- 生成复杂赛道 ---
        self.track_gen = TrackGenerator(resolution=0.2)
        self.generate_complex_track()
        self.get_logger().info('Complex Global Path Publisher Initialized.')

    def generate_complex_track(self):
        # 1. 起步直道 (50m)
        self.track_gen.add_straight(50.0)
        # 2. 90度急左转 (R=15m)
        self.track_gen.add_turn(90, 15.0, 'left')
        # 3. 短直道 (20m)
        self.track_gen.add_straight(20.0)
        # 4. 90度急右转 (R=15m)
        self.track_gen.add_turn(90, 15.0, 'right')
        # 5. 长直道 (40m)
        self.track_gen.add_straight(400.0)
        # 6. 180度大回环 (R=25m)
        self.track_gen.add_turn(180, 205.0, 'left')
        # 7. S弯 (连续弯道)
        self.track_gen.add_turn(45, 200.0, 'right')
        self.track_gen.add_turn(45, 200.0, 'left')
        # 8. 终点直道
        self.track_gen.add_straight(15000.0)
        
        self.track_points = self.track_gen.get_path()

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for p in self.track_points:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            
            # 计算四元数 (虽然通常 Controller 自己会算，但发出去更好看)
            yaw = p[2]
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)
            
            path.poses.append(pose)
            
        self.pub_path.publish(path)

def main():
    rclpy.init()
    rclpy.spin(GlobalPathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()