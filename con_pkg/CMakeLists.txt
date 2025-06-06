cmake_minimum_required(VERSION 3.8)
project(con_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces for actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavCommand.action"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  src/con.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()