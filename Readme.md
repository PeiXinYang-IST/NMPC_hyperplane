# NMPC 追踪器

一个基于 ROS2 的非线性模型预测控制（NMPC）追踪器，具有 **Lattice 横向规划**、**基于 SFC 的走廊约束**、**超平面障碍物避障** 和 **ESO 扰动估计** 功能。

## 系统架构

**当前实现**: RTK 路径 + Lattice 横向规划 + NMPC + ESO

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  RTK 路径   │ ──▶│   Lattice   │ ──▶│    NMPC     │ ──▶│    ESO      │ ──▶│   车辆控制   │
│ (全局路径)  │    │  (横向规划)  │    │   (纵向控制) │    │  (扰动估计)  │    │             │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                          │
                    ┌─────┴─────┐
                    │  SFC 走廊  │
                    │ (约束条件)  │
                    └───────────┘
```

**核心组件**:

1. **路径输入**: 从 ROS 话题接收 RTK 路径（全局路径）
2. **Lattice 横向规划**: 基于候选轨迹的横向轨迹规划，支持障碍物避让
3. **SFC 约束**: 用于安全约束的空间填充曲线走廊
4. **NMPC**: 使用 Acados 的非线性模型预测控制（纵向控制）
5. **ESO**: 扩展状态观测器，用于扰动估计和补偿

### 关于 SFC

**历史问题**: SFC 约束之前因 **过多的松弛变量** 而失败，导致目标函数急剧上升并使优化停滞。

**解决方案**: 适当调整的松弛变量界限结合 SFC 走廊约束，可以在不过度增加代价的情况下实现稳定的障碍物避障。

---

## 日志系统

本项目为安全关键的轨迹追踪提供两种不同的接口：

### 1. 基于超平面的接口

**用例**: 具有法向量约束的简单圆形障碍物避障。

- 障碍物约束: `nx * (x - ox) + ny * (y - oy) >= r`
- 计算最近的障碍物并生成法向量约束
- 通过自动生成的 C 代码与 Acados 集成

**关键文件**:
- `include/hyperplane_util.hpp` - 超平面工具函数
- `scripts/generate_c.py` - Acados C 代码生成
- `scripts/c_generated_code/` - 生成的约束和模型代码

### 2. 基于 SFC 的接口

**用例**: 基于走廊的路径追踪，具有矩形安全约束。

- 沿参考路径生成空间填充曲线走廊
- 每个走廊由四边形凸多边形组成
- 为 NMPC 约束提供 `A` 矩阵和 `b` 向量

**关键文件**:
- `include/sfc_generator.hpp` / `src/sfc_generator.cpp` - SFC C++ 实现
- `scripts/sfc_lib/generator.py` - SFC Python 实现
- 通过 `config/parameters.yaml` 启用: `sfc.enable: true`

---

## 当前实现: Lattice 横向规划 + NMPC

**当前系统使用 Lattice 横向规划结合 NMPC 进行轨迹跟踪:**

- **Lattice 横向规划**: 基于候选轨迹的横向规划，采样多个横向偏移轨迹并选择最优路径
- **NMPC**: 使用 Acados 进行纵向控制，包含以下权重配置:
  - 位置跟踪权重
  - 速度跟踪权重
  - 加速度平滑权重

**关键文件**:
- `include/lattice_planner.hpp` - C++ Lattice 横向规划器
- `scripts/planner/generate_c_planner.py` - 规划器 Acados C 代码生成
- `scripts/controller/generate_c_controller.py` - 控制器 Acados C 代码生成

## 演示

参见 `pic` 文件夹中的演示视频。

### 超平面
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image.png)
### SFC
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image1.jpg)

## 功能特性

- **NMPC 控制**: 使用 Acados 实现高性能非线性模型预测控制
- **障碍物感知**: 使用 DBSCAN 聚类进行障碍物检测和追踪
- **横向规划**: 基于 Lattice 的横向轨迹规划，支持候选轨迹选择
- **轨迹生成**: GCopter 轨迹规划库，支持 MINCO 轨迹生成和 SFC 走廊约束
- **两种避障接口**: 基于超平面和基于 SFC 的轨迹追踪
- **路径平滑**: 基于 FEM 的后端优化，实现平滑轨迹
- **扩展状态观测器 (ESO)**: 基于 ESO 的扰动估计，支持线加速度和角加速度独立补偿
  - 线性 ESO: 估计线速度扰动（动态摩擦、坡度等）
  - 角速度 ESO: 估计角速度扰动（执行误差、模型不确定性等）

## 环境要求

- Ubuntu 22.04
- ROS2 Humble/Foxy
- C++20
- Acados（带 HPIPM 和 BLASFEO）
- CMake 3.8+

---

## 安装配置

### 1. 安装 ROS2

```bash
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-visualization-msgs ros-humble-tf2 \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-ros ros-humble-launch
sudo apt install ccache
sudo apt install mold
```

### 2. 安装 Acados

```bash
git clone https://github.com/acados/acados.git
cd acados
mkdir -p build && cd build
cmake .. -DACADOS_WITH_OPENMP=OFF
make install
```

### 3. 配置 Acados 路径

```bash
# 将 <YOUR_ACADOS_PATH> 替换为您的实际 Acados 安装路径
export ACADOS_INSTALL_DIR="<YOUR_ACADOS_PATH>"

# 添加到 ~/.bashrc 以实现持久化
echo "export ACADOS_INSTALL_DIR=\"<YOUR_ACADOS_PATH>\"" >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<YOUR_ACADOS_PATH>/lib' >> ~/.bashrc
source ~/.bashrc
```

### 4. 生成 Acados C 代码

```bash

# 生成规划器 C 代码
cd ../planner
python3 generate_c_planner.py
```

### 5. 构建项目

```bash
cd ..
colcon build
source install/setup.bash
```

---

## 快速开始

### 选项 1: C++ 版本

```bash
# 终端 1: 启动追踪器
ros2 launch nmpc_tracker nmpc_test_launch.py

# 终端 2: 运行仿真演示
cd scripts
python3 test.py
```

### 选项 2: Python 演示

```bash
cd scripts
python3 test.py
```

---

## 配置参数

可以通过启动文件或命令行调整关键参数：

### Lattice 规划参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `lattice.path_resolution` | 0.2 | 路径采样分辨率 (m) |
| `lattice.lookahead_dist` | 20.0 | 前视距离 (m) |
| `lattice.num_samples` | 7 | 横向采样数量 |
| `lattice.sample_width` | 0.5 | 横向采样宽度 (m) |
| `lattice.max_width` | 3.0 | 最大横向偏移 (m) |
| `lattice.collision_radius` | 0.8 | 碰撞检测半径 (m) |

### 障碍物避障接口参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `sfc.enable` | false | 启用 SFC 接口（false = 超平面） |
| `sfc.robot_radius` | 0.3 | 机器人走廊半径 |
| `sfc.search_radius` | 1.0 | 障碍物搜索半径 |
| `sfc.longitudinal_length` | 0.5 | 走廊纵向长度 |
| `obstacle_avoidance.base_margin` | 0.8 | 安全裕度 (m) |

### NMPC 参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `nmpc_config.ref_velocity` | 5.0 | 参考速度 (m/s) |
| `nmpc_config.control_loop_ms` | 50 | 控制周期 (ms) |

### 感知参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `perception.dbscan_eps` | 1.2 | DBSCAN 聚类 epsilon |
| `perception.dbscan_min_pts` | 3 | 每簇最小点数 |

### 机器人限制

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `robot_limits.max_linear_velocity` | 6.0 | 最大速度 (m/s) |
| `robot_limits.max_angular_velocity` | 2.5 | 最大角速度 (rad/s) |

### ESO 参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `eso.enable` | true | 是否启用 ESO |
| `eso.omega_linear` | 5.0 | 线性 ESO 观测带宽 (rad/s) |
| `eso.omega_angular` | 1.0 | 角速度 ESO 观测带宽 (rad/s) |
| `eso.b0_linear` | 1.0 | 线性 ESO 标称增益 |
| `eso.b0_angular` | 1.0 | 角速度 ESO 标称增益 |

**ESO 工作原理**:
- 线性 ESO: 基于输入加速度 `ax` 观测速度扰动 `dist_lin`，补偿公式: `ax_comp = ax_raw - dist_lin / b0_linear`
- 角速度 ESO: 基于输入角加速度 `w_acc = (w_cmd - last_w_cmd) / dt` 观测角速度扰动 `dist_ang`，补偿公式: `w_comp = w_raw - dist_ang / b0_angular`

---

## 接口切换

### 使用超平面接口（默认）

超平面接口默认启用，无需额外配置。

### 使用 SFC 接口

在 `config/parameters.yaml` 中启用 SFC：

```yaml
sfc:
  enable: true
  robot_radius: 0.3
  search_radius: 1.0
  longitudinal_length: 0.5
```


## 项目结构

```
nmpc_tracker/
├── include/              # 头文件
│   ├── nmpc_tracker_node.hpp
│   ├── nmpc_visualizer.hpp
│   ├── sfc_generator.hpp      # SFC 接口
│   ├── hyperplane_util.hpp    # 超平面接口
│   ├── astar_planner.hpp      # A* 规划器
│   ├── lattice_planner.hpp    # 横向规划器
│   ├── eso.hpp
│   ├── DBSCAN.hpp
│   └── gcopter/               # GCopter 轨迹规划库
│       ├── gcopter.hpp
│       ├── flatness.hpp
│       ├── minco.hpp
│       ├── sfc_gen.hpp
│       └── ...
├── src/                  # 源文件
│   ├── nmpc_tracker_node.cpp
│   ├── sfc_generator.cpp      # SFC 实现
│   └── main.cpp
├── scripts/              # Python 脚本
│   ├── simulation.py
│   ├── path_publisher.py
│   ├── cubic_spline.py
│   ├── astar_planner.py       # A* Python 实现
│   ├── rtk_path.py            # RTK 路径读取
│   ├── plot_log_trajectory.py # 日志轨迹绘图
│   ├── sfc_lib/               # SFC Python 库
│   │   ├── generator.py
│   │   ├── config.py
│   │   └── utils.py
│   ├── controller/             # 控制器 Acados C 代码生成
│   │   └── generate_c_controller.py
│   ├── planner/                # 规划器 Acados C 代码生成
│   │   └── generate_c_planner.py
│   ├── test.py
│   └── generate_c.py
├── datest/               # 数据测试与分析
│   ├── nihe.py               # 速度映射拟合
│   ├── niheyaw.py            # 角速度映射拟合
│   ├── vis_ang.py            # 可视化角度数据
│   ├── compare_outputs.py    # 输出对比
│   ├── control.py            # 控制分析
│   └── deal_path.py          # 路径处理
├── launch/               # 启动文件
├── pic/                  # 演示视频和图片
├── config/
│   └── parameters.yaml   # 配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```


## 系统测试速度映射步骤

1. 在 pnc 文件夹中修改 x 和 z 速度
2. 开启 DDS bridge
3. 开启 RTK 程序，获取底层速度估计
4. 开启 PNC launch
5. 从 RTK 终端打印日志导出，并保存记录，文件名为当前 PNC 发送速度数据
6. 准备好数据后送入 `nihe.py`

```python
file_map = {
    '0.4log.txt': 0.4,
    '0.35log.txt': 0.35,
    '0.3log.txt': 0.3,
    '0.25log.txt': 0.25,
    '0.2log.txt': 0.2,
    '0.15log.txt': 0.15,
    '0.1log.txt': 0.1
}
```

---

## 1. 系统输入与架构

系统主要由 `nmpc tracker node` 核心节点组成，输入源分为两部分：

* **定位数据 (RTK):**
* 接收位置信息：`x`, `y`
* 接收航向角：`yaw`

* **遥控指令 (Remote Control):**
* 线速度 $v_x$：目前状态较稳定。
* 角速度 $v_{yaw}$：**需滤波处理**。当前直接使用 IMU 原始数据（裸数据），存在噪点。


---

## 2. 数据预处理

在数据进入控制算法前，需进行映射与过滤：

* **速度映射:** 线速度 $v_x$ 采用线性映射公式：

$$v_{out} = 3.17 \times v_{in} + 0.10$$


* **死区处理:** 为防止零点漂移，对角速度 $v_{yaw}$ 设置死区：
* **死区阈值:** $0.33 \, \text{rad/s}$



---

## 3. NMPC 调速修改指南

若需调整车辆的运动速度范围，请按以下步骤操作：

### 3.1 代码内部逻辑修改

修改 `dynamic step dist` 相关计算代码，确保 `max_step` 和 `min_step` 与速度对应：

对应node中代码为：
double dynamic_step_dist = max_step - curve_ratio * (max_step - min_step); 
    if (dynamic_step_dist < 0.1) dynamic_step_dist = 0.1; 

* $max\_vel = max\_step \times \frac{1}{dt}$
* $min\_vel = min\_step \times \frac{1}{dt}$

### 3.2 配置文件 (YAML) 修改

1. 修改 `yaml` 文件中的参考速度 `ref_vel` (即 `max_vel`)。
2. 修改 `yaml` 文件中的最大速度限制（振幅）参数 `max_vel`。
3. 目前测试其他部分较稳定不要轻易修改
---

## 4. 性能优化与故障排除

当 **NMPC 跟踪效果较差**（如出现明显震荡或不稳定）时，可尝试以下方案：

* **权重调整:** 修改 `planner.py` 或 `generate.py` 中的各部分权重。
* **优化策略:** * **减小振荡:** 通常通过 **降低位置跟踪权重** (Position Tracking Weight) 来减少系统的频繁摆动，提高行驶轨迹的平滑度。

## 关于测试PNC是否稳定
1.可视化中查看前端选择的路径（蓝色路径中选择出的绿色路径）是否较贴合全局轨迹
2.nmpc的预测轨迹是否贴合前端绿色路径
3.查看终端中的打印 总时间是否超过30ms 若超过30ms则时间复杂度较高（目前orin上一次循环应为10ms左右）

## 5. 脚本说明

### scripts/ 目录

| 脚本 | 说明 |
|------|------|
| `simulation.py` | 仅做仿真，发布 odom 和模拟障碍物点云。**实际跑不要运行！** 我们的 odom 是从 rtk node 中获取的 |
| `path_publisher.py` | 同上，仅做仿真测试程序是否正常。**实际跑不要运行！** |
| `rtk_path.py` | 读取 RTK 预先记录的路径，并截取局部路径做发布 |
| `plot_log_trajectory.py` | 先获取 RTK 的终端 log 数据至 txt 中（格式同 `scripts/log.txt`，不同可修改脚本中的正则表达式），之后运行得到插值之后的 `path.txt`。**这里的 path.txt 要供 rtk_path.py 读取，作为 global path** |
| `astar_planner.py` | Python 版 A* 路径规划器 |
| `cubic_spline.py` | 三次样条插值 |
| `sfc_generator.py` | SFC 走廊生成器 |

### scripts/controller/ 目录

| 脚本 | 说明 |
|------|------|
| `generate_c_controller.py` | 控制器 Acados C 代码生成 |

### scripts/planner/ 目录

| 脚本 | 说明 |
|------|------|
| `generate_c_planner.py` | 规划器 Acados C 代码生成 |

### datest/ 目录

| 脚本 | 说明 |
|------|------|
| `nihe.py` | 速度映射拟合（线性映射: v_out = 3.17 * v_in + 0.10） |
| `niheyaw.py` | 角速度映射拟合 |
| `vis_ang.py` | 可视化角度数据 |
| `compare_outputs.py` | 输出对比分析 |
| `control.py` | 控制分析脚本 |
| `deal_path.py` | 路径数据处理 |