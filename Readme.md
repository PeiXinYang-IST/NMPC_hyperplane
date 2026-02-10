# NMPC Tracker

A ROS2 Nonlinear Model Predictive Control (NMPC) tracker featuring **two obstacle avoidance interfaces**: hyperplane-based and SFC-based (Space-Filling Curve corridor), with **A* planning and backend optimization**.

## Two Obstacle Avoidance Interfaces

This project provides two distinct interfaces for safety-critical trajectory tracking:

### 1. Hyperplane Interface (基于超平面的接口)

**Use Case**: Simple circular obstacle avoidance with normal vector constraints.

- Obstacle constraint: `nx * (x - ox) + ny * (y - oy) >= r`
- Computes the closest obstacle and generates normal vector constraints
- Integrates with Acados through auto-generated C code

**Key Files**:
- `include/hyperplane_util.hpp` - Hyperplane utility functions
- `scripts/generate_c.py` - Acados C code generation
- `scripts/c_generated_code/` - Generated constraint and model code

### 2. SFC Interface (基于SFC的接口)

**Use Case**: Corridor-based path tracking with rectangular safety constraints.

- Generates space-filling curve corridors around the reference path
- Each corridor consists of 4-sided convex polygons
- Provides `A` matrix and `b` vector for NMPC constraints

**Key Files**:
- `include/sfc_generator.hpp` / `src/sfc_generator.cpp` - SFC C++ implementation
- `scripts/sfc_lib/generator.py` - SFC Python implementation
- Enable via `config/parameters.yaml`: `sfc.enable: true`

---

## Current Implementation: A* + Backend Optimization

**The current implementation uses A* path planning combined with backend optimization for path smoothing:**

- **A* Planning**: Grid-based global path planner with Octile distance heuristic
- **Backend Optimization**: FEM (Finite Element Method) path smoothing with configurable weights:
  - `smooth_w_data`: Data fidelity weight (0.45)
  - `smooth_w_smooth`: Smoothness weight (0.40)
  - `smooth_w_curvature`: Curvature weight (0.40)

**Key Files**:
- `include/astar_planner.hpp` - C++ A* planner with memory pool optimization
- `scripts/astar_planner.py` - Python A* implementation

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
- **Two Avoidance Interfaces**: Hyperplane-based and SFC-based trajectory tracking
- **Path Smoothing**: FEM-based backend optimization for smooth trajectories
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
sudo apt install ccache
sudo apt install mold
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

### A* Planning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `astar.resolution` | 0.4 | Grid resolution (m) |
| `astar.heuristic_weight` | 1.2 | Heuristic weight for A* |
| `astar.reference_cost_weight` | 2.0 | Reference path attraction |
| `astar.turning_weight` | 2.5 | Turning penalty |

### Backend Optimization Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `astar.smooth_data_weight` | 0.45 | Data fidelity weight |
| `astar.smooth_smooth_weight` | 0.35 | Smoothness weight |
| `astar.smooth_curvature_weight` | 0.35 | Curvature weight |

### Obstacle Avoidance Interface Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sfc.enable` | false | Enable SFC interface (false = hyperplane) |
| `sfc.robot_radius` | 0.3 | Robot radius for corridor |
| `sfc.search_radius` | 1.0 | Obstacle search radius |
| `sfc.longitudinal_length` | 0.5 | Corridor longitudinal length |
| `obstacle_avoidance.base_margin` | 0.8 | Safety margin (m) |

### NMPC Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `nmpc_config.ref_velocity` | 5.0 | Reference velocity (m/s) |
| `nmpc_config.control_loop_ms` | 50 | Control loop period (ms) |

### Perception Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `perception.dbscan_eps` | 1.2 | DBSCAN epsilon for clustering |
| `perception.dbscan_min_pts` | 3 | Minimum points per cluster |

### Robot Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_limits.max_linear_velocity` | 6.0 | Max velocity (m/s) |
| `robot_limits.max_angular_velocity` | 2.5 | Max angular velocity (rad/s) |

---

## Switching Between Interfaces

### Using Hyperplane Interface (Default)

The hyperplane interface is enabled by default. No additional configuration needed.

### Using SFC Interface

Enable SFC in `config/parameters.yaml`:

```yaml
sfc:
  enable: true
  robot_radius: 0.3
  search_radius: 1.0
  longitudinal_length: 0.5
```

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
│   ├── sfc_generator.hpp      # SFC interface
│   ├── hyperplane_util.hpp    # Hyperplane interface
│   ├── astar_planner.hpp      # A* planner
│   ├── eso.hpp
│   └── DBSCAN.hpp
├── src/                  # Source files
│   ├── nmpc_tracker_node.cpp
│   ├── sfc_generator.cpp      # SFC implementation
│   └── main.cpp
├── scripts/              # Python scripts
│   ├── simulation.py
│   ├── path_publisher.py
│   ├── cubic_spline.py
│   ├── astar_planner.py       # A* Python implementation
│   ├── sfc_lib/               # SFC Python library
│   │   ├── generator.py
│   │   ├── config.py
│   │   └── utils.py
│   ├── test.py
│   └── generate_c.py
├── launch/               # Launch files
├── pic/                  # Demo videos and images
├── config/
│   └── parameters.yaml   # Configuration file
├── CMakeLists.txt
├── package.xml
└── README.md
```
