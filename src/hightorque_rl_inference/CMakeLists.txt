# ================================================================================
# CMakeLists.txt for hightorque_rl_inference package
# hightorque_rl_inference 功能包的 CMake 构建配置文件
#
# This file defines the build configuration for the RL inference demo package.
# It sets up dependencies, include directories, and build targets for both the
# library and executable node.
# 
# 本文件定义了强化学习推理演示功能包的构建配置。
# 它设置了依赖项、包含目录，以及库和可执行节点的构建目标。
#
# Developed by: 高擎机电 (HighTorque Robotics)
# ================================================================================

cmake_minimum_required(VERSION 3.8)
project(hightorque_rl_inference)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

add_definitions(-DPLATFORM_ARM)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/rknn
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(INFERENCE_LIBS rknnrt)

add_library(${PROJECT_NAME}
  src/hightorque_rl_inference.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${INFERENCE_LIBS}
  ${YAMLCPP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rosgraph_msgs
  ament_index_cpp
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rosgraph_msgs
  ament_index_cpp
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  config_example.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY policy/
  DESTINATION share/${PROJECT_NAME}/policy
)

ament_package()

