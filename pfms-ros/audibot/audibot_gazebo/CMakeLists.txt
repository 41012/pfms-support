cmake_minimum_required(VERSION 3.5)
project(audibot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

include_directories(include)

add_library(AudibotInterfacePlugin SHARED
  src/AudibotInterfacePlugin.cpp
)
target_link_libraries(AudibotInterfacePlugin
  ignition-gazebo6::core
  ignition-plugin1::register
)

ament_export_libraries(AudibotInterfacePlugin)
install(
  TARGETS AudibotInterfacePlugin
  DESTINATION lib
)

install(
  DIRECTORY
    config
    launch
    rviz
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
