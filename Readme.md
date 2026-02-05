### acados生成c code
cd scripts
python3 generate_c.py

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


N_PARAM 25 
N_HORIZON 60
DT 0.05
REF_VEL 7.0  
