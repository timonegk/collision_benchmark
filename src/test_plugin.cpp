#include <reach/interfaces/evaluator.h>
#include <reach/plugin_utils.h>
#include <boost_plugin_loader/plugin_loader.h>
#include <iostream>

int main() {
  boost_plugin_loader::PluginLoader loader;
  std::vector<std::string> plugin_libraries;
  loader.search_libraries = {"benchmark_reach_plugins"};

  std::map<std::string, double> test_map = {
      {"shoulder_pan_joint", 0.0},
      {"shoulder_lift_joint", 0.0},
      {"elbow_joint", 0.0},
      {"wrist_1_joint", 0.0},
      {"wrist_2_joint", 0.0},
      {"wrist_3_joint", 0.0},
      {"endo_first_joint", 0.0},
      {"endo_second_joint_first_dof", 0.0},
      {"endo_second_joint_second_dof", 0.0}
  };
  reach::Evaluator::ConstPtr evaluator;
  {
    auto factory = loader.createInstance<reach::EvaluatorFactory>("ReachabilityEvaluator");
    YAML::Node y;
    evaluator = factory->create(y);
    std::cout << "A" << evaluator->calculateScore(test_map) << std::endl;
  }
  std::cout << "B" << evaluator->calculateScore(test_map) << std::endl;
}