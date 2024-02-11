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
#add_executable(ugv_remote src/ugv_remote.cpp)
#target_link_libraries(ugv_remote ${catkin_LIBRARIES})

add_executable(gazebo_connect src/gazebo_connect.cpp)
ament_target_dependencies(gazebo_connect rclcpp gazebo_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros)
#ament_target_dependencies(gazebo_connect rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros)

#add_executable(odo src/odo.cpp)
#target_link_libraries(odo ${catkin_LIBRARIES} rt pthread )

add_executable(reach src/reach.cpp)
ament_target_dependencies(reach rclcpp std_srvs geometry_msgs nav_msgs)

add_executable(tf_transform_republisher src/tf_transform_republisher.cpp)
ament_target_dependencies(tf_transform_republisher rclcpp geometry_msgs tf2 tf2_ros)

install(TARGETS
  gazebo_connect
  reach
  tf_transform_republisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
