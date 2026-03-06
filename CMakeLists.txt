cmake_minimum_required(VERSION 3.8)
project(nmpc_tracker)

# --- 编译器设置 ---
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()
find_program(MOLD_LINKER mold)
if(MOLD_LINKER)
    add_link_options("-fuse-ld=mold")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 依赖查找 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# --- 关键修改路径设置 ---
# 1. 设置 Acados 安装路径 (根据您的环境保持不变)
set(ACADOS_INSTALL_DIR "/mnt/c/Users/yang/Downloads/acados")

# 2. 分别定义 Planner (5D) 和 Controller (3D) 的生成代码路径
#    注意：这里假设 scripts 文件夹在您的 ROS 包根目录下
set(PLANNER_CODE_DIR "${PROJECT_SOURCE_DIR}/scripts/planner/c_generated_code")
set(CONTROLLER_CODE_DIR "${PROJECT_SOURCE_DIR}/scripts/controller/c_generated_code")

# 3. 包含头文件路径 (必须包含两个目录)
include_directories(include 
    ${PLANNER_CODE_DIR} 
    ${CONTROLLER_CODE_DIR}
    ${ACADOS_INSTALL_DIR}/include 
    ${ACADOS_INSTALL_DIR}/include/blasfeo/include 
    ${ACADOS_INSTALL_DIR}/include/hpipm/include
)
link_directories(${ACADOS_INSTALL_DIR}/lib)

# --- 收集源文件 ---

# 4.1 收集 Planner (5D) 的源文件
file(GLOB PLANNER_TOP_C "${PLANNER_CODE_DIR}/*.c")
# 排除 main 文件防止冲突
list(FILTER PLANNER_TOP_C EXCLUDE REGEX ".*main.*")
file(GLOB_RECURSE PLANNER_MODEL_C 
    "${PLANNER_CODE_DIR}/*_model/*.c" 
    "${PLANNER_CODE_DIR}/*_cost/*.c" 
    "${PLANNER_CODE_DIR}/*_constraints/*.c"
)

# 4.2 收集 Controller (3D) 的源文件
file(GLOB CONTROLLER_TOP_C "${CONTROLLER_CODE_DIR}/*.c")
# 排除 main 文件
list(FILTER CONTROLLER_TOP_C EXCLUDE REGEX ".*main.*")
file(GLOB_RECURSE CONTROLLER_MODEL_C 
    "${CONTROLLER_CODE_DIR}/*_model/*.c" 
    "${CONTROLLER_CODE_DIR}/*_cost/*.c" 
    "${CONTROLLER_CODE_DIR}/*_constraints/*.c"
)

# --- 编译可执行文件 ---

add_executable(nmpc_node
  src/main.cpp
  src/nmpc_tracker_node.cpp  
  src/sfc_generator.cpp
  # 5. 将两套 Acados 代码都加入编译
  ${PLANNER_TOP_C}    
  ${PLANNER_MODEL_C}
  ${CONTROLLER_TOP_C}
  ${CONTROLLER_MODEL_C}
)

ament_target_dependencies(nmpc_node 
    rclcpp nav_msgs sensor_msgs geometry_msgs visualization_msgs 
    tf2 tf2_geometry_msgs tf2_ros
)

target_link_libraries(nmpc_node 
    acados hpipm blasfeo m 
    tf2_geometry_msgs::tf2_geometry_msgs
)

# --- 安装规则 ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS nmpc_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/simulation.py
  scripts/path_publisher.py
  scripts/cubic_spline.py
  scripts/rtk_path.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  scripts/full_ordered_path.txt
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()