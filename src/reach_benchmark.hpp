#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

class ReachBenchmark {
 public:
  ReachBenchmark();
  void run();
 private:
  YAML::Node config_;
  std::string config_name_;
  boost::filesystem::path results_dir_;
};