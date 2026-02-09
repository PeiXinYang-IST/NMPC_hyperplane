### demo见pic文件夹

![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image.png)  

### acados生成c code

#### cd scripts
#### python3 generate_c.py
#### echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/mnt/c/Users/yang/Downloads/acados/lib' >> ~/.bashrc   换成自己的

### 编译

#### cd ..
#### colcon build

#### source install/setup.bash

## launch
### cpp版
#### ros2 launch nmpc_tracker nmpc_test_launch.py
### python demo
#### cd scripts
#### python3 test.py

### 2026 2.6
#### 1.测试中发现如果对于实际速度与cmd_vel差距较大时，nmpc会有问题（可能与热启动有关），需要做系统辨识（cmd_vel与实际速度的映射关系，二阶拟合）
#### 2.加入eso拓张状态观测器处理动态摩擦以及上下坡场景
#### 3.对于运行时间长后X和Y的数值过大带来数值不稳定导致求解失败的问题转为local frame求解
#### 4.在当前横向误差大于一定阈值（路宽的一定比例之后 开启recovery mode，desired vel设置为0.5m/s，全力拉回
#### （以上全部前提都必须下位机较严格执行速度）
### TODO:
#### 1.系统辨识cmd_vel与实际速度的映射关系，二阶拟合
#### 2.最大角速度 最大角加速度 最大加速度 最大速度
#### 3.对于上下坡场景的处理
#### 4.验证eso可行性，参数设置
#### 5.参数设置