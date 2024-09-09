cmake_minimum_required(VERSION 3.7)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(reach REQUIRED)
find_package(reach_ros REQUIRED)
find_package(bio_ik REQUIRED)
find_package(relaxed_ik_cpp REQUIRED)

add_library(
  collision_benchmark_reach SHARED
  src/benchmark_ik_solver.cpp
  src/relaxed_ik_goals.cpp
  src/bio_ik_goals.cpp
)
target_include_directories(collision_benchmark_reach PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(collision_benchmark_reach PUBLIC reach::reach)
ament_target_dependencies(collision_benchmark_reach PUBLIC bio_ik relaxed_ik_cpp reach_ros)

add_library(collision_benchmark_reach_plugin SHARED
    src/reach_plugin.cpp)
target_link_libraries(collision_benchmark_reach_plugin PUBLIC collision_benchmark_reach)

install(TARGETS collision_benchmark_reach collision_benchmark_reach_plugin DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY scripts USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scenarios DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(bio_ik reach_ros relaxed_ik_cpp)
ament_export_libraries(collision_benchmark_reach)
ament_package()
