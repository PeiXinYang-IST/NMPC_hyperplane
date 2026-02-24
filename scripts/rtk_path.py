#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import os

class GlobalPathPublisher(Node):
    def __init__(self):
        super().__init__('global_path_publisher')
        
        # --- 参数设置 ---
        # 路径文件绝对路径 (请修改为你实际的 txt 文件路径)
        # 假设文件格式为每行: x, y (或者 x, y, yaw)
        self.declare_parameter('path_file', '/mnt/c/Users/yang/Downloads/Tracker/scripts/path.txt')
        self.path_file_path = self.get_parameter('path_file').get_parameter_value().string_value
        
        self.lookahead_dist = 30.0  # 前瞻距离 (米)，即发布机器人前方多少米的路径
        self.publish_rate = 0.1    # 发布频率 (秒)，即 10Hz
        self.global_frame_id = 'map' # 全局坐标系 ID

        # --- 1. 路径发布者 ---
        # 发布话题 /ref_path，用于 MPC 或 pure pursuit 跟踪
        self.pub_path = self.create_publisher(Path, '/ref_path', 10)
        
        # --- 2. 里程计订阅者 ---
        # 订阅 /odom 获取机器人实时位置
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # --- 3. 定时器 ---
        self.timer = self.create_timer(self.publish_rate, self.publish_path)
        
        # --- 4. 加载全局路径 ---
        self.global_path = self.load_path_from_file(self.path_file_path)
        
        # 机器人当前状态
        self.robot_x = None
        self.robot_y = None

        if self.global_path is not None:
            self.get_logger().info(f'Loaded {len(self.global_path)} points from {self.path_file_path}')
        else:
            self.get_logger().error(f'Failed to load path from {self.path_file_path}')

    def load_path_from_file(self, file_path):
        points = []
        if not os.path.exists(file_path):
            self.get_logger().error(f"Path file not found: {file_path}")
            return None

        try:
            with open(file_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    # 过滤空行、注释行（#）以及包含字母的表头行
                    if not line or line.startswith('#') or any(c.isalpha() for c in line):
                        continue
                    
                    if ',' in line:
                        parts = line.split(',')
                    else:
                        parts = line.split()
                    
                    if len(parts) >= 2:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            yaw = float(parts[2]) if len(parts) > 2 else 0.0
                            points.append([x, y, yaw])
                        except ValueError:
                            # 进一步防止由于非数字内容导致的转换失败
                            continue
            
            return np.array(points)

        except Exception as e:
            self.get_logger().error(f"Error reading file: {e}")
            return None

    def odom_callback(self, msg):
        """ 更新机器人当前位置 """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def publish_path(self):
        """ 定时切片并发布局部路径 """
        
        # 1. 检查是否具备发布条件
        if self.global_path is None or len(self.global_path) == 0:
            return
        if self.robot_x is None or self.robot_y is None:
            # 可选：如果没收到 odom，可以发布整个路径或第一段路径用于可视化
            return

        # 2. 寻找全局路径中距离机器人最近的点 (Search Nearest Point)
        # 计算欧氏距离
        dx = self.global_path[:, 0] - self.robot_x
        dy = self.global_path[:, 1] - self.robot_y
        dists = np.hypot(dx, dy)
        
        # 获取最小距离的索引
        min_idx = np.argmin(dists)

        # 3. 截取路径 (Path Slicing)
        # 我们不仅需要最近点，还需要从最近点开始往后的一段路径
        # 计算路径点的平均间距 (分辨率)，用于估算需要取多少个点
        # 这里为了简单，假设分辨率相对均匀，或者直接根据距离累加来找终点
        
        current_dist = 0.0
        end_idx = min_idx
        
        # 向后搜索直到达到 lookahead_dist
        for i in range(min_idx, len(self.global_path) - 1):
            p1 = self.global_path[i]
            p2 = self.global_path[i+1]
            segment_dist = np.hypot(p2[0]-p1[0], p2[1]-p1[1])
            current_dist += segment_dist
            end_idx = i + 1
            if current_dist >= self.lookahead_dist:
                break
        
        # 如果到了路径末尾距离还不够，就取到末尾
        path_segment = self.global_path[min_idx : end_idx + 1]

        # 4. 构建并发布 ROS Path 消息
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.global_frame_id
        
        for p in path_segment:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = p[0]
            pose_stamped.pose.position.y = p[1]
            pose_stamped.pose.position.z = 0.0 # 假设是 2D 平面
            
            # 处理朝向 (Yaw -> Quaternion)
            # 如果源文件没有 yaw 数据，这里 p[2] 可能为 0，或者你可以根据路径几何实时计算切线方向
            yaw = p[2] 
            # 简单的欧拉角转四元数 (Z轴旋转)
            half_yaw = yaw * 0.5
            pose_stamped.pose.orientation.z = np.sin(half_yaw)
            pose_stamped.pose.orientation.w = np.cos(half_yaw)
            
            path_msg.poses.append(pose_stamped)
            
        self.pub_path.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()