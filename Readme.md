### acados生成c code
cd scripts
python3 generate_c.py

### 编译
cd ..
colcon build

source install/setup.bash

### launch
ros2 launch nmpc_tracker nmpc_test_launch.py
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/%5BWARN_COPY%20MODE%5D%20Figure%201%20(Ubuntu-22.04)%202026-02-03%2020-17-32.mp4)  