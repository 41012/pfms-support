cmake_minimum_required(VERSION 2.8.13)
project(gazebo_tf)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg gazebo_msgs geometry_msgs tf)

## Declare ROS messages and services
add_service_files(FILES SetGoal.srv SetGoals.srv GetMinDistToGoal.srv SetCell.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Include the headers and library directory needed
find_package(pipes)

message ( STATUS "Include Dirs ${catkin_INCLUDE_DIRS}" )
message ( STATUS "Include Libs ${catkin_LIBRARIES}" )

#Create executables
add_executable(ugv_remote src/ugv_remote.cpp)
target_link_libraries(ugv_remote ${catkin_LIBRARIES})

add_executable(gazebo_connect src/gazebo_connect.cpp)
target_link_libraries(gazebo_connect ${catkin_LIBRARIES})

add_executable(odo src/odo.cpp)
target_link_libraries(odo ${catkin_LIBRARIES} rt pthread )

add_executable(reach src/reach.cpp)
target_link_libraries(reach ${catkin_LIBRARIES} )

## Add cmake target dependencies of the executable
## This enforces generating messages before building these executables
add_dependencies(odo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(reach ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/cleanup
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
