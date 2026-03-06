#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import os
import struct

class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')

        # 1. 路径发布者
        self.pub_path = self.create_publisher(Path, '/ref_path', 10)

        # 2. 点云发布者 (左右边界)
        self.pub_cloud = self.create_publisher(PointCloud2, '/scan_cloud', 10)

        # 3. 里程计订阅者 (获取当前位置)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 4. 定时器 (控制发布频率，例如 10Hz)
        self.timer = self.create_timer(0.05, self.publish_path)

        # 5. 读取完整路径文件和边界文件
        self.load_full_path()

        # 状态变量
        self.robot_x = None
        self.robot_y = None
        self.lookahead_dist = 150.0  # 截取距离 (米)

        self.get_logger().info('Local Window Path Publisher Initialized.')

    def load_full_path(self):
        """从文件读取完整路径和边界"""
        # 从环境变量获取工作空间路径
        ws_path = os.environ.get('AMENT_PREFIX_PATH', '')
        if ws_path:
            # 取第一个路径，去掉 install 部分
            ws_path = ws_path.split('/install/')[0] if '/install/' in ws_path else ws_path
            scripts_dir = os.path.join(ws_path, 'scripts')
        else:
            # 备用：从脚本所在目录获取
            scripts_dir = os.path.dirname(os.path.abspath(__file__))

        # 中心线文件
        central_file = os.path.join(scripts_dir, 'full_ordered_path.txt')
        left_file = os.path.join(scripts_dir, 'left_boundary.txt')
        right_file = os.path.join(scripts_dir, 'right_boundary.txt')

        self.get_logger().info(f'Loading path from: {central_file}')

        # 读取中心线
        points = []
        with open(central_file, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    points.append([x, y])
        points = np.array(points)

        # 计算 yaw (航向角) 和累积距离
        n = len(points)
        yaws = np.zeros(n)
        cum_dist = np.zeros(n)
        for i in range(1, n):
            dx = points[i, 0] - points[i-1, 0]
            dy = points[i, 1] - points[i-1, 1]
            yaws[i] = np.arctan2(dy, dx)
            cum_dist[i] = cum_dist[i-1] + np.hypot(dx, dy)

        self.track_points = np.column_stack([points, yaws, cum_dist])
        self.resolution = np.mean(np.diff(cum_dist))
        self.get_logger().info(f'Loaded {n} central path points, resolution: {self.resolution:.4f} m')

        # 读取左边界
        self.left_points = self.load_boundary_file(left_file, 'left')
        # 读取右边界
        self.right_points = self.load_boundary_file(right_file, 'right')

    def load_boundary_file(self, file_path, name):
        """加载边界文件"""
        if not os.path.exists(file_path):
            self.get_logger().warn(f'{name} boundary file not found: {file_path}')
            return np.array([])

        points = []
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    points.append([x, y])

        pts = np.array(points)
        self.get_logger().info(f'Loaded {len(pts)} {name} boundary points')
        return pts

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

        # 5. 发布边界点云
        self.publish_boundary_cloud(closest_idx, end_idx)

    def publish_boundary_cloud(self, start_idx, end_idx):
        """发布边界点云 (左右边界)"""
        # 收集边界点 (只取窗口范围内的点)
        cloud_points = []

        # 左边界点
        if len(self.left_points) > 0:
            # 简单处理：取全部或按距离截取
            for i in range(len(self.left_points)):
                cloud_points.append(self.left_points[i])

        # 右边界点
        if len(self.right_points) > 0:
            for i in range(len(self.right_points)):
                cloud_points.append(self.right_points[i])

        if not cloud_points:
            return

        cloud_points = np.array(cloud_points)

        # 创建 PointCloud2 消息
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'map'
        cloud_msg.height = 1
        cloud_msg.width = len(cloud_points)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True

        # 打包点数据
        buf = bytearray()
        for pt in cloud_points:
            buf.extend(struct.pack('fff', float(pt[0]), float(pt[1]), 0.0))
        cloud_msg.data = bytes(buf)

        self.pub_cloud.publish(cloud_msg)

def main():
    rclpy.init()
    rclpy.spin(GlobalPathPublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()