### acados生成c code
cd scripts
python3 generate_c.py

### 编译
cd ..
colcon build

source install/setup.bash

### launch
ros2 launch nmpc_tracker nmpc_test_launch.py
