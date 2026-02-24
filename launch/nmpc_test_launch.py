import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    acados_lib = '/mnt/c/Users/yang/Downloads/acados/lib'
    gen_code_lib = '/mnt/c/Users/yang/Downloads/Tracker/scripts/c_generated_code'
    config_path = '/mnt/c/Users/yang/Downloads/Tracker/config/parameters.yaml'
    
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
        Node(
            package='nmpc_tracker', 
            executable='nmpc_node', 
            output='screen', 
            env=env,
            parameters=[config_path] # 加载参数文件
        ),
        
        # Python 仿真节点
        Node(package='nmpc_tracker', executable='simulation.py', output='screen', env=env),
        
        # Python 路径发布节点
        Node(package='nmpc_tracker', executable='path_publisher.py', output='screen', env=env),
        # Node(package='nmpc_tracker', executable='rtk_path.py', output='screen', env=env),

        Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    # This forces RViz to start with a clean, default config
    arguments=['-d', ''] 
)

    ])