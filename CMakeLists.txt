cmake_minimum_required(VERSION 3.7)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(benchmark_node src/benchmark_node.cpp src/benchmark.cpp
  src/benchmark_logger.cpp)
ament_target_dependencies(
  benchmark_node
  moveit_core
  moveit_ros_planning_interface
  rclcpp
)
install(TARGETS benchmark_node DESTINATION lib/${PROJECT_NAME})

add_executable(motion_bench_maker_importer src/motion_bench_maker_importer.cpp src/yaml.cpp)
target_link_libraries(motion_bench_maker_importer yaml-cpp)
ament_target_dependencies(
  motion_bench_maker_importer
  yaml-cpp
  rclcpp
  ament_index_cpp
  moveit_msgs
)
install(TARGETS motion_bench_maker_importer DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY motion_bench_maker DESTINATION share/${PROJECT_NAME})

ament_package()
