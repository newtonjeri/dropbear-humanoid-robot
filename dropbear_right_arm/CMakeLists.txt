cmake_minimum_required(VERSION 3.8)
project(dropbear_right_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install python executables
install(
  PROGRAMS
    ${PROJECT_NAME}/hand_trajectory_publisher.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch meshes urdf rviz config
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
