#include "reachability_evaluator.hpp"
#include <iostream>

namespace benchmark {

double ReachabilityEvaluator::calculateScore(const std::map<std::string, double> &) const {
  return 1.0;
}

reach::Evaluator::ConstPtr ReachabilityEvaluatorFactory::create(const YAML::Node &) const {
  return std::make_shared<ReachabilityEvaluator>();
}

}  // namespace benchmark