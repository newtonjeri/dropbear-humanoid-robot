cmake_minimum_required(VERSION 3.8)
project(simple_closed_loop_mechanism)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)


find_package(angles REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)


include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

# Libraries
add_library(${PROJECT_NAME} SHARED
  src/gazebo_sim_hw.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  angles
  controller_manager
  control_toolbox
  gazebo_dev
  gazebo_ros
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
)
pluginlib_export_plugin_description_file(gazebo_sim_hw simple_closed_loop_mechanism_plugin.xml)


## Install
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ament_cmake
  angles
  controller_manager
  gazebo_dev
  gazebo_ros
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
)

ament_export_libraries(
  ${PROJECT_NAME}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


# Move folders into the shared folder of the project
install(
  DIRECTORY
    config
    launch
    rviz
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

# install python executables
install(
  PROGRAMS
    scripts/jtc_publisher_node.py
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
