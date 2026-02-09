#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class TrackGenerator:
    """ 赛道生成器：支持直线、圆弧拼接 (保持不变) """
    def __init__(self, resolution=0.1):
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
        
        # 1. 路径发布者
        self.pub_path = self.create_publisher(Path, '/ref_path', 10)
        
        # 2. 里程计订阅者 (获取当前位置)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 3. 定时器 (控制发布频率，例如 10Hz)
        self.timer = self.create_timer(0.05, self.publish_path) 
        
        # 4. 生成赛道
        self.resolution = 0.1
        self.track_gen = TrackGenerator(resolution=self.resolution)
        self.generate_complex_track()
        
        # 状态变量
        self.robot_x = None
        self.robot_y = None
        self.lookahead_dist = 150.0 # 截取距离 (米)
        
        self.get_logger().info('Local Window Path Publisher Initialized.')

    def generate_complex_track(self):
        # ... (赛道生成逻辑保持不变) ...
        self.track_gen.add_straight(50.0)
        self.track_gen.add_turn(90, 15.0, 'left')
        self.track_gen.add_straight(20.0)
        self.track_gen.add_turn(90, 15.0, 'right')
        self.track_gen.add_straight(400.0)
        self.track_gen.add_turn(180, 205.0, 'left')
        self.track_gen.add_turn(45, 200.0, 'right')
        self.track_gen.add_turn(45, 200.0, 'left')
        self.track_gen.add_straight(15000.0)
        
        self.track_points = self.track_gen.get_path()

    def odom_callback(self, msg):
        """ 更新机器人当前位置 """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def publish_path(self):
        # 如果还没收到里程计数据，暂不发布（或者发布全路径，视需求而定）
        if self.robot_x is None or self.robot_y is None:
            return

        # 1. 寻找最近点索引
        # 计算所有点到当前位置的欧氏距离
        # (注意：如果路径点非常多，数万个点，这里可以使用 KD-Tree 优化，或者只搜索上一次索引附近的窗口)
        dx = self.track_points[:, 0] - self.robot_x
        dy = self.track_points[:, 1] - self.robot_y
        dists = np.hypot(dx, dy)
        closest_idx = np.argmin(dists)

        # 2. 计算截取范围
        # 需要截取的点数 = 距离 / 分辨率
        num_points = int(self.lookahead_dist / self.resolution)
        
        end_idx = closest_idx + num_points
        
        # 3. 截取路径 (处理数组越界情况，Python切片会自动处理越界，但为了逻辑清晰)
        # 获取从最近点开始的切片
        path_segment = self.track_points[closest_idx : end_idx]

        # 4. 构建 ROS 消息
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # 注意：这里我们假设全局路径和odom是在同一个坐标系下 (通常是 'odom' 或 'map')
        # 如果你在 rviz 里看，Fixed Frame 要选对
        path_msg.header.frame_id = 'map' 
        
        for p in path_segment:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            
            # 简单的 Yaw 转 Quaternion
            yaw = p[2]
            pose.pose.orientation.z = np.sin(yaw / 2.0)
            pose.pose.orientation.w = np.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
            
        self.pub_path.publish(path_msg)

def main():
    rclpy.init()
    rclpy.spin(GlobalPathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()