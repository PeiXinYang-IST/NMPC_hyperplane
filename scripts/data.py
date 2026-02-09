#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder_node')

        # ================= 配置区域 =================
        self.cmd_topic = '/cmd_vel'      # cmd_vel 话题名
        self.odom_topic = '/odom'        # odom 话题名
        self.file_path = 'velocity_log.txt' # 保存的文件名
        self.sample_rate = 20.0          # 采样频率 (Hz)，建议与控制频率一致
        # ===========================================

        # 初始化数据缓存变量
        self.current_cmd_vx = 0.0
        self.current_cmd_wz = 0.0
        self.current_odom_vx = 0.0
        self.current_odom_wz = 0.0

        # 订阅 cmd_vel
        self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        # 订阅 odom
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # 打开文件 (使用 'w' 模式覆盖，或 'a' 模式追加)
        try:
            self.file_handle = open(self.file_path, 'w')
            # 可选：写入表头，方便人类阅读（如果只需要纯数据，可以注释掉下面这行）
            # self.file_handle.write("cmd_vx cmd_wz odom_vx odom_wz\n")
            self.get_logger().info(f'开始记录数据到: {os.path.abspath(self.file_path)}')
        except Exception as e:
            self.get_logger().error(f'无法打开文件: {e}')
            return

        # 创建定时器进行记录
        self.timer = self.create_timer(1.0 / self.sample_rate, self.timer_callback)

    def cmd_callback(self, msg):
        """回调函数：更新最新的指令速度"""
        self.current_cmd_vx = msg.linear.x
        self.current_cmd_wz = msg.angular.z

    def odom_callback(self, msg):
        """回调函数：更新最新的里程计速度"""
        # 注意：Odometry 消息结构中，速度在 msg.twist.twist 下
        self.current_odom_vx = msg.twist.twist.linear.x
        self.current_odom_wz = msg.twist.twist.angular.z

    def timer_callback(self):
        """定时器：将当前缓存的数据写入文件"""
        # 格式化字符串：保留4位小数，空格分隔
        line = f"{self.current_cmd_vx:.4f} {self.current_cmd_wz:.4f} {self.current_odom_vx:.4f} {self.current_odom_wz:.4f}\n"
        
        try:
            self.file_handle.write(line)
            # 刷新缓冲区，确保数据实时写入硬盘，防止程序崩溃数据丢失
            self.file_handle.flush()
        except Exception as e:
            self.get_logger().error(f'写入文件失败: {e}')

    def destroy_node(self):
        """节点销毁时关闭文件"""
        if hasattr(self, 'file_handle') and not self.file_handle.closed:
            self.file_handle.close()
            self.get_logger().info('文件已关闭，录制结束。')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = DataRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()