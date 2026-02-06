### acados生成c code
cd scripts
python3 generate_c.py
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/mnt/c/Users/yang/Downloads/acados/lib' >> ~/.bashrc
### 编译
cd ..
colcon build

source install/setup.bash

## launch
### cpp版
ros2 launch nmpc_tracker nmpc_test_launch.py
### python demo
cd scripts
python3 test.py

