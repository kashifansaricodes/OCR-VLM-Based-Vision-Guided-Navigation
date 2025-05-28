cmake_minimum_required(VERSION 3.8)
project(vlm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ocr_pkg REQUIRED)
find_package(con_pkg REQUIRED)

# Install Python modules
install(
  DIRECTORY src
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS
    src/vlm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()