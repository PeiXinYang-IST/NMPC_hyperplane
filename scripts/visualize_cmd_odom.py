#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from math import atan2, asin

class CmdOdomVisualizer(Node):
    def __init__(self):
        super().__init__('cmd_odom_visualizer')
        self.cmd_vx = deque(maxlen=200)
        self.odom_vx = deque(maxlen=200)
        self.cmd_wz = deque(maxlen=200)
        self.odom_yaw = deque(maxlen=200)
        self.time_cmd = deque(maxlen=200)
        self.time_odom = deque(maxlen=200)

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.cmd_times = deque(maxlen=50)
        self.odom_times = deque(maxlen=50)
        self.cmd_hz = deque(maxlen=200)
        self.odom_hz = deque(maxlen=200)
        self.hz_time = deque(maxlen=200)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))

    def cmd_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time_cmd.append(t)
        self.cmd_vx.append(msg.linear.x)
        self.cmd_wz.append(msg.angular.z)

        self.cmd_times.append(t)
        if len(self.cmd_times) > 1:
            hz = (len(self.cmd_times) - 1) / (self.cmd_times[-1] - self.cmd_times[0])
            self.cmd_hz.append(hz)
            self.hz_time.append(t)

    def odom_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time_odom.append(t)
        self.odom_vx.append(msg.twist.twist.linear.x)

        q = msg.pose.pose.orientation
        yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.odom_yaw.append(yaw)

        self.odom_times.append(t)
        if len(self.odom_times) > 1:
            hz = (len(self.odom_times) - 1) / (self.odom_times[-1] - self.odom_times[0])
            self.odom_hz.append(hz)

        self.update_plot()

    def update_plot(self):
        if len(self.cmd_hz) < 2 or len(self.odom_hz) < 2:
            return

        # Plot 1: Hz
        self.ax1.clear()
        n_hz = min(len(self.hz_time), len(self.cmd_hz), len(self.odom_hz))
        self.ax1.plot(list(self.hz_time)[-n_hz:], list(self.cmd_hz)[-n_hz:], 'r-', label='cmd_vel Hz')
        self.ax1.plot(list(self.hz_time)[-n_hz:], list(self.odom_hz)[-n_hz:], 'b-', label='odom Hz')
        self.ax1.set_ylabel('Frequency (Hz)')
        self.ax1.set_ylim(0, 20)
        self.ax1.legend()
        self.ax1.grid(True)

        # Plot 2: Linear velocity
        n = min(len(self.time_cmd), len(self.odom_vx))
        if n > 1:
            self.ax2.clear()
            self.ax2.plot(list(self.time_cmd)[-n:], list(self.cmd_vx)[-n:], 'r-', label='cmd_vel vx')
            self.ax2.plot(list(self.time_odom)[-n:], list(self.odom_vx)[-n:], 'b-', label='odom vx')
            self.ax2.set_ylabel('Linear Velocity (m/s)')
            self.ax2.legend()
            self.ax2.grid(True)

        # Plot 3: Empty
        self.ax3.clear()
        self.ax3.set_xlabel('Time (s)')
        self.ax3.grid(True)

        plt.tight_layout()
        plt.pause(0.01)

if __name__ == '__main__':
    rclpy.init()
    viz = CmdOdomVisualizer()
    rclpy.spin(viz)
    viz.destroy_node()
    rclpy.shutdown()
