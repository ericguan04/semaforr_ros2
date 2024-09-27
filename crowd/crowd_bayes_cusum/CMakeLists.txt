cmake_minimum_required(VERSION 3.8)
project(crowd_bayes_cusum)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Include Python setup
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/bayes_filter.py  # List your Python scripts
  src/learn.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()