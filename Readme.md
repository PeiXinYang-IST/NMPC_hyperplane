# NMPC 追踪器

一个基于 ROS2 的非线性模型预测控制（NMPC）追踪器，具有 **A* 路径规划**、**后端优化**、**基于 SFC 的走廊约束** 和 **超平面障碍物避障** 功能。

## 系统架构

**当前实现**: A* + 后端优化 + SFC 约束 + NMPC

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   A* 规划器  │ ──▶│  FEM 平滑   │ ──▶│  SFC 走廊    │ ──▶│    NMPC     │
│  (全局路径)  │    │ (后端优化)  │    │  (约束条件)  │    │   (控制)    │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
```

**核心组件**:

1. **A* 规划**: 基于网格的全局路径规划器，使用八边形距离启发式
2. **后端优化**: FEM（有限元法）路径平滑
3. **SFC 约束**: 用于安全约束的空间填充曲线走廊
4. **NMPC**: 使用 Acados 的非线性模型预测控制

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

## 当前实现: A* + 后端优化

**当前实现使用 A* 路径规划结合后端优化进行路径平滑:**

- **A* 规划**: 基于网格的全局路径规划器，使用八边形距离启发式
- **后端优化**: FEM（有限元法）路径平滑，具有可配置的权重:
  - `smooth_w_data`: 数据保真权重 (0.45)
  - `smooth_w_smooth`: 平滑权重 (0.40)
  - `smooth_w_curvature`: 曲率权重 (0.40)

**关键文件**:
- `include/astar_planner.hpp` - C++ A* 规划器，带内存池优化
- `scripts/astar_planner.py` - Python A* 实现

## 演示

参见 `pic` 文件夹中的演示视频。

### 超平面
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image.png)
### SFC
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image1.jpg)

## 功能特性

- **NMPC 控制**: 使用 Acados 实现高性能非线性模型预测控制
- **障碍物感知**: 使用 DBSCAN 聚类进行障碍物检测和追踪
- **路径规划**: 带有动态障碍物避障的 A* 全局规划器
- **两种避障接口**: 基于超平面和基于 SFC 的轨迹追踪
- **路径平滑**: 基于 FEM 的后端优化，实现平滑轨迹
- **扩展状态观测器**: 基于 ESO 的扰动估计（动态摩擦、坡度）

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
cd scripts
python3 generate_c.py
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

### A* 规划参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `astar.resolution` | 0.4 | 网格分辨率 (m) |
| `astar.heuristic_weight` | 1.2 | A* 启发式权重 |
| `astar.reference_cost_weight` | 2.0 | 参考路径吸引力 |
| `astar.turning_weight` | 2.5 | 转弯惩罚 |

### 后端优化参数

| 参数 | 默认值 | 描述 |
|-----------|---------|-------------|
| `astar.smooth_data_weight` | 0.45 | 数据保真权重 |
| `astar.smooth_smooth_weight` | 0.35 | 平滑权重 |
| `astar.smooth_curvature_weight` | 0.35 | 曲率权重 |

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

---

## 已知问题与待办事项

- **速度映射**: 如果实际速度与 `cmd_vel` 差异显著，可能需要进行系统辨识（对 `cmd_vel` 到实际速度映射进行二阶拟合）
- **坡度处理**: 动态摩擦和坡度场景需要扩展 ESO
- **大坐标值**: 长时间运行后转换为局部坐标系以防止数值不稳定
- **恢复模式**: 当横向误差超过阈值时，启用低速度（0.5 m/s）恢复模式

### 待办列表

- [ ] 对 `cmd_vel` 到实际速度映射进行系统辨识
- [ ] 设置最大角速度、加速度和速度限制
- [ ] 改进坡度处理
- [ ] 验证 ESO 可行性和参数调整
- [ ] 参数优化

---

## 项目结构

```
nmpc_tracker/
├── include/              # 头文件
│   ├── nmpc_tracker_node.hpp
│   ├── nmpc_visualizer.hpp
│   ├── sfc_generator.hpp      # SFC 接口
│   ├── hyperplane_util.hpp    # 超平面接口
│   ├── astar_planner.hpp      # A* 规划器
│   ├── eso.hpp
│   └── DBSCAN.hpp
├── src/                  # 源文件
│   ├── nmpc_tracker_node.cpp
│   ├── sfc_generator.cpp      # SFC 实现
│   └── main.cpp
├── scripts/              # Python 脚本
│   ├── simulation.py
│   ├── path_publisher.py
│   ├── cubic_spline.py
│   ├── astar_planner.py       # A* Python 实现
│   ├── sfc_lib/               # SFC Python 库
│   │   ├── generator.py
│   │   ├── config.py
│   │   └── utils.py
│   ├── test.py
│   └── generate_c.py
├── launch/               # 启动文件
├── pic/                  # 演示视频和图片
├── config/
│   └── parameters.yaml   # 配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```
