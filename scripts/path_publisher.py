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
        self.timer = self.create_timer(2.0, self.publish_path) 
        self.get_logger().info('Global Path Publisher Initialized.')

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for i in range(1500):
            pose = PoseStamped()
            x = float(i) * 0.2
            pose.pose.position.x = x
            # y = 3.0 * sin(0.1 * x)
            pose.pose.position.y = 3.0 * np.sin(0.1 * x) 
            path.poses.append(pose)
            
        self.pub_path.publish(path)
        # self.get_logger().info('Global path published.')

def main():
    rclpy.init()
    rclpy.spin(GlobalPathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# ref vel 7m/s
# 0.25 5m/s
# 0.2 4.2m/s
# 0.15 3m/s
# 0.1 2m/s
