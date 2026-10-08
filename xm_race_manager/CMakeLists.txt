cmake_minimum_required(VERSION 3.8)
project(xm_race_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANGCXX)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 可执行文件：源头文件 -> 目标名
add_executable(race_manager_node src/race_manager_node.cpp)
ament_target_dependencies(race_manager_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs std_msgs)

# 装进 install 空间：ros2 run 才找得到
install(TARGETS race_manager_node
  DESTINATION lib/${PROJECT_NAME})

# launch 目录整个拷过去：ros2 launch 才找得到
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
