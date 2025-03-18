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
