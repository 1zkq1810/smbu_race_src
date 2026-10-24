cmake_minimum_required(VERSION 3.8)
project(rmul2027_sentry_task)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pb_rm_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sentry_task_node src/sentry_task_node.cpp)
target_compile_features(sentry_task_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  sentry_task_node
  rclcpp
  rclcpp_action
  nav2_msgs
  pb_rm_interfaces
  geometry_msgs
)

install(TARGETS sentry_task_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
