cmake_minimum_required(VERSION 3.8)
project(ObstacleAvoider)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(obstacle_avoider_algorithm src/obstacle_avoider_algorithm.cpp)
ament_target_dependencies(obstacle_avoider_algorithm rclcpp geometry_msgs sensor_msgs)

install(TARGETS obstacle_avoider_algorithm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
