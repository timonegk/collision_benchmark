cmake_minimum_required(VERSION 3.7)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(benchmark_node src/benchmark_node.cpp src/benchmark.cpp
  src/benchmark_logger.cpp)
ament_target_dependencies(
  benchmark_node
  moveit_core
  moveit_ros_planning_interface
  rclcpp
)

install(TARGETS benchmark_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
