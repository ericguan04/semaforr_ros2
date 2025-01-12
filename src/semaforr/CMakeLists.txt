cmake_minimum_required(VERSION 3.8)
project(semaforr)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add Python3 development package
find_package(Python3 REQUIRED COMPONENTS Development)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CrowdModel.msg"
  DEPENDENCIES std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include/semaforr/
  ${Python3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c/
)

## Declare a C++ executable
file(
  GLOB_RECURSE
  source_files
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/include/semaforr/*.h
)

add_executable(semaforr_node ${source_files})

# Link against Python3 libraries
target_link_libraries(semaforr_node ${Python3_LIBRARIES})

ament_target_dependencies(semaforr_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "visualization_msgs"
)

# Install targets
install(TARGETS semaforr_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
)

# Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )`

ament_package()