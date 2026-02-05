import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- 路径配置 (请根据您的环境确认路径) ---
    acados_lib = '/mnt/c/Users/yang/Downloads/acados/lib'
    gen_code_lib = '/mnt/c/Users/yang/Downloads/Tracker/scripts/c_generated_code'
    
    # Python 路径设置，确保能找到自定义的 python 脚本
    python_path = os.environ.get('PYTHONPATH', '')
    ros_python_path = '/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages'
    new_python_path = f"{ros_python_path}:{python_path}"

    env = {
        'LD_LIBRARY_PATH': f"{acados_lib}:{gen_code_lib}:" + os.environ.get('LD_LIBRARY_PATH', ''),
        'PYTHONPATH': new_python_path,
        'ROS_LOG_DIR': '/tmp',
        'PYTHONUNBUFFERED': '1'
    }

    return LaunchDescription([
        # 1. 核心 NMPC 节点 (使用您刚刚编译好的 C++ 节点)
        Node(
            package='nmpc_tracker',
            executable='nmpc_node',
            output='screen',
            parameters=[{
                'nmpc_config.ref_velocity': 6.0,       # 提高速度进行压测
                'obstacle_avoidance.base_margin': 0.8, # 稍微激进一点
                'perception.dbscan_min_pts': 2         # 对稀疏点云更敏感
            }],
            env=env
        ),
        
        # 2. 压力测试管理器 (替代原本的 simulation.py 和 path_publisher.py)
        # 注意：这里直接调用 python 脚本，假设它已被安装或位于包路径下
        # 如果未安装，可以使用 execute_process 直接运行 python3 脚本路径
        Node(
            package='nmpc_tracker',
            executable='stress_test_manager.py', 
            output='screen',
            env=env
        ),
        
        # 3. 可视化
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
            # 您可以保存一个默认的 .rviz 配置并在这里加载
            # arguments=['-d', '/path/to/your/config.rviz'] 
        ),

        # 4. 性能监控 (rqt_plot)
        # 监控求解时间和速度，观察是否有剧烈波动
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            arguments=['/nmpc/solve_time', '/cmd_vel/linear/x'],
            output='log'
        )
    ])