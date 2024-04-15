#include <filesystem>
#include "reach_benchmark.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <reach/reach_study.h>

ReachBenchmark::ReachBenchmark() :
    config_(YAML::LoadFile(ament_index_cpp::get_package_share_directory("benchmark") +
        "/config/reach_config.yaml")),
    config_name_("reach_config"),
    results_dir_("/tmp/reach_results") {
}

void ReachBenchmark::run() {
  setenv("REACH_PLUGINS", "reach_ros_plugins", false);
  reach::runReachStudy(config_, config_name_, results_dir_, false);
  std::filesystem::remove_all(results_dir_.string());
}