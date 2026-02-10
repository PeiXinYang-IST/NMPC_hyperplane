cmake_minimum_required(VERSION 3.8)
project(nmpc_tracker)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(ACADOS_INSTALL_DIR "/mnt/c/Users/yang/Downloads/acados")
set(GEN_CODE_DIR "${PROJECT_SOURCE_DIR}/scripts/c_generated_code")

include_directories(include ${GEN_CODE_DIR} ${ACADOS_INSTALL_DIR}/include 
    ${ACADOS_INSTALL_DIR}/include/blasfeo/include ${ACADOS_INSTALL_DIR}/include/hpipm/include)
link_directories(${ACADOS_INSTALL_DIR}/lib)

file(GLOB TOP_C "${GEN_CODE_DIR}/*.c")
list(FILTER TOP_C EXCLUDE REGEX ".*main.*")
file(GLOB_RECURSE MODEL_C "${GEN_CODE_DIR}/*_model/*.c" "${GEN_CODE_DIR}/*_cost/*.c" "${GEN_CODE_DIR}/*_constraints/*.c")

add_executable(nmpc_node
  src/main.cpp
  src/nmpc_tracker_node.cpp  
  src/sfc_generator.cpp
  ${TOP_C}    # Acados 接口实现
  ${MODEL_C}  # 具体的模型/代价/约束函数实现
)
ament_target_dependencies(nmpc_node 
    rclcpp nav_msgs sensor_msgs geometry_msgs visualization_msgs 
    tf2 tf2_geometry_msgs tf2_ros
)

target_link_libraries(nmpc_node 
    acados hpipm blasfeo m 
    tf2_geometry_msgs::tf2_geometry_msgs
)
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
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()