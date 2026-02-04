import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 路径定义
    acados_lib = '/mnt/c/Users/yang/Downloads/acados/lib'
    gen_code_lib = '/mnt/c/Users/yang/Downloads/Tracker/scripts/c_generated_code'
    
    # 获取当前环境中的 PYTHONPATH 并添加 ROS2 路径
    python_path = os.environ.get('PYTHONPATH', '')
    ros_python_path = '/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages'
    new_python_path = f"{ros_python_path}:{python_path}"

    env = {
        'LD_LIBRARY_PATH': f"{acados_lib}:{gen_code_lib}:" + os.environ.get('LD_LIBRARY_PATH', ''),
        'PYTHONPATH': new_python_path, # 显式设置 Python 路径
        'ROS_LOG_DIR': '/tmp',
        'PYTHONUNBUFFERED': '1'
    }

    return LaunchDescription([
        # C++ NMPC 节点
        Node(package='nmpc_tracker', executable='nmpc_node', output='screen', env=env),
        
        # Python 仿真节点
        Node(package='nmpc_tracker', executable='simulation.py', output='screen', env=env),
        
        # Python 路径发布节点
        Node(package='nmpc_tracker', executable='path_publisher.py', output='screen', env=env),
        
        # RViz2 可视化
        Node(package='rviz2', executable='rviz2', output='screen')
    ])