cmake_minimum_required(VERSION 3.5)
project(autodrive_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ackermann_to_autodrive_node.cpp
  src/wheel_odometry_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN autodrive_bridge::AckermannToAutodriveNode
  EXECUTABLE ackermann_to_autodrive_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN autodrive_bridge::WheelOdometryNode
  EXECUTABLE wheel_odometry_node
)

ament_export_include_directories(include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
