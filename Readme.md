# NMPC Tracker

A ROS2 Nonlinear Model Predictive Control (NMPC) tracker with hyperplane-based obstacle avoidance, featuring Acados optimization, DBSCAN clustering, and A* planning.

## Demo

See the `pic` folder for demonstration videos.

### hyperplane
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image.png)
### SFC
![运行示意图](https://github.com/PeiXinYang-IST/NMPC_hyperplane/blob/main/pic/image1.jpg)

## Features

- **NMPC Control**: High-performance nonlinear model predictive control using Acados
- **Obstacle Perception**: DBSCAN clustering for obstacle detection and tracking
- **Path Planning**: A* global planner with dynamic obstacle avoidance
- **Hyperplane-based Control**: Safety-critical trajectory tracking
- **Extended State Observer**: ESO-based disturbance estimation (dynamic friction, slope)

## Requirements

- Ubuntu 22.04
- ROS2 Humble/Foxy
- C++20
- Acados (with HPIPM and BLASFEO)
- CMake 3.8+

---

## Setup

### 1. Install ROS2

```bash
sudo apt update
sudo apt install ros-humble-rclcpp ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-visualization-msgs ros-humble-tf2 \
  ros-humble-tf2-geometry-msgs ros-humble-tf2-ros ros-humble-launch
```

### 2. Install Acados

```bash
git clone https://github.com/acados/acados.git
cd acados
mkdir -p build && cd build
cmake .. -DACADOS_WITH_OPENMP=OFF
make install
```

### 3. Configure Acados Path

```bash
# Replace <YOUR_ACADOS_PATH> with your actual acados installation path
export ACADOS_INSTALL_DIR="<YOUR_ACADOS_PATH>"

# Add to ~/.bashrc for persistence
echo "export ACADOS_INSTALL_DIR=\"<YOUR_ACADOS_PATH>\"" >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<YOUR_ACADOS_PATH>/lib' >> ~/.bashrc
source ~/.bashrc
```

### 4. Generate Acados C Code

```bash
cd scripts
python3 generate_c.py
```

### 5. Build the Project

```bash
cd ..
colcon build
source install/setup.bash
```

---

## QuickStart

### Option 1: C++ Version

```bash
# Terminal 1: Launch the tracker
ros2 launch nmpc_tracker nmpc_test_launch.py

# Terminal 2: Run simulation demo
cd scripts
python3 test.py
```

### Option 2: Python Demo

```bash
cd scripts
python3 test.py
```

---

## Configuration

Key parameters can be tuned via the launch file or command line:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `nmpc_config.ref_velocity` | 5.0 | Reference velocity (m/s) |
| `nmpc_config.control_loop_ms` | 50 | Control loop period (ms) |
| `perception.dbscan_eps` | 1.2 | DBSCAN epsilon for clustering |
| `perception.dbscan_min_pts` | 3 | Minimum points per cluster |
| `obstacle_avoidance.base_margin` | 0.8 | Safety margin (m) |
| `robot_limits.max_linear_velocity` | 6.0 | Max velocity (m/s) |
| `robot_limits.max_angular_velocity` | 2.5 | Max angular velocity (rad/s) |

---

## Known Issues & TODO

- **Speed Mapping**: If actual velocity differs significantly from `cmd_vel`, system identification may be needed (second-order fitting for `cmd_vel` to actual velocity mapping)
- **Slope Handling**: ESO extension needed for dynamic friction and slope scenarios
- **Large Coordinate Values**: Convert to local frame after extended operation to prevent numerical instability
- **Recovery Mode**: When lateral error exceeds threshold, enable recovery mode with low velocity (0.5 m/s)

### TODO List
- [ ] System identification for `cmd_vel` to actual velocity mapping
- [ ] Set max angular velocity, acceleration, and velocity limits
- [ ] Slope handling improvements
- [ ] Validate ESO feasibility and parameter tuning
- [ ] Parameter optimization

---

## Project Structure

```
nmpc_tracker/
├── include/              # Header files
│   ├── nmpc_tracker_node.hpp
│   ├── nmpc_visualizer.hpp
│   ├── sfc_generator.hpp
│   ├── astar_planner.hpp
│   ├── eso.hpp
│   ├── hyperplane_util.hpp
│   └── DBSCAN.hpp
├── src/                  # Source files
│   ├── nmpc_tracker_node.cpp
│   ├── sfc_generator.cpp
│   └── main.cpp
├── scripts/              # Python scripts
│   ├── simulation.py
│   ├── path_publisher.py
│   ├── cubic_spline.py
│   ├── test.py
│   └── generate_c.py
├── launch/               # Launch files
├── pic/                  # Demo videos and images
├── CMakeLists.txt
├── package.xml
└── README.md
```
