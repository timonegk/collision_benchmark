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
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(reach REQUIRED)
find_package(reach_ros REQUIRED)
find_package(boost_plugin_loader REQUIRED)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

add_executable(benchmark_node
  src/benchmark_node.cpp
  src/benchmark.cpp
  src/benchmark_logger.cpp
  src/benchmark_visualizer.cpp
  src/planning_benchmark.cpp
  src/utils.cpp
  src/reach_benchmark.cpp
)
ament_target_dependencies(
  benchmark_node
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools
  rclcpp
  tf2_eigen
  rviz_rendering
  reach_ros
)
target_link_libraries(benchmark_node yaml-cpp reach::reach)
install(TARGETS benchmark_node DESTINATION lib/${PROJECT_NAME})

#add_executable(motion_bench_maker_importer src/motion_bench_maker_importer.cpp src/yaml.cpp)
#target_link_libraries(motion_bench_maker_importer yaml-cpp)
#ament_target_dependencies(
#  motion_bench_maker_importer
#  yaml-cpp
#  rclcpp
#  ament_index_cpp
#  moveit_msgs
#)

add_library(benchmark_reach_plugins SHARED
    src/plugins.cpp
    src/reachability_evaluator.cpp
    src/benchmark_moveit_ik_solver.cpp
)
target_include_directories(benchmark_reach_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(benchmark_reach_plugins PUBLIC reach::reach)
ament_target_dependencies(benchmark_reach_plugins PUBLIC
    boost_plugin_loader
    rclcpp
    reach_ros
    moveit_msgs
)

install(TARGETS benchmark_reach_plugins DESTINATION lib)

add_executable(test_plugin src/test_plugin.cpp)
target_include_directories(test_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(test_plugin boost_plugin_loader reach_ros)
target_link_libraries(test_plugin reach::reach)
install(TARGETS test_plugin DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scenarios DESTINATION share/${PROJECT_NAME})

install(DIRECTORY scripts DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
