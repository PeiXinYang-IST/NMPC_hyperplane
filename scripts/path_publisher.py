#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')
        self.pub_path = self.create_publisher(Path, '/ref_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path) 
        self.get_logger().info('Complex Global Path Publisher Initialized.')

    def get_road_y(self, x):
        """ 必须与 SimulationEnv 中的方程一致 """
        # y = 4sin(0.1x) + 2cos(0.05x)
        return 4.0 * np.sin(0.1 * x) + 2.0 * np.cos(0.05 * x)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        # 生成 300米 长的路径
        for i in range(3000):
            pose = PoseStamped()
            x = float(i) * 0.2 # 分辨率 0.1m
            pose.pose.position.x = x
            pose.pose.position.y = self.get_road_y(x)
            
            # 这里虽然不严格需要计算朝向(z,w)，但有些 Path Follower 可能需要
            # 为了简单起见，这里只发位置，朝向由控制器计算切线决定
            path.poses.append(pose)
            
        self.pub_path.publish(path)

def main():
    rclpy.init()
    rclpy.spin(GlobalPathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()