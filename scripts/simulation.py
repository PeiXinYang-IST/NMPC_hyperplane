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
        
        # [x, y, yaw, v, omega]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_and_publish)
        
        # 静态障碍物
        self.obstacles = [[5.0, 0.5], [10.0, -0.5], [15.0, 0.3]]

    def cmd_cb(self, msg):
        # 仿真环境作为执行器，直接接收计算好的速度
        self.state[3] = msg.linear.x
        self.state[4] = msg.angular.z

    def update_and_publish(self):
        # 运动学积分
        self.state[0] += self.state[3] * np.cos(self.state[2]) * self.dt
        self.state[1] += self.state[3] * np.sin(self.state[2]) * self.dt
        self.state[2] += self.state[4] * self.dt

        # 发布里程计
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        
        yaw = self.state[2]
        odom.pose.pose.orientation.z = np.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.angular.z = self.state[4]
        self.pub_odom.publish(odom)

        # 发布点云障碍物
        points = []
        for obs in self.obstacles:
            for ang in np.linspace(0, 2*np.pi, 8):
                points.append([obs[0] + 0.3*np.cos(ang), obs[1] + 0.3*np.sin(ang), 0.0])
        pc_msg = pc2.create_cloud_xyz32(odom.header, points)
        self.pub_cloud.publish(pc_msg)

def main():
    rclpy.init()
    rclpy.spin(SimulationEnv())
    rclpy.shutdown()

if __name__ == '__main__':
    main()