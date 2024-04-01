cmake_minimum_required(VERSION 3.5)
project(gazebo_tf)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#Create executables
add_executable(gazebo_connect src/gazebo_connect.cpp)
ament_target_dependencies(gazebo_connect rclcpp gazebo_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(reach src/reach.cpp)
ament_target_dependencies(reach rclcpp std_srvs geometry_msgs nav_msgs)

install(TARGETS
  gazebo_connect
  reach
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    models
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
