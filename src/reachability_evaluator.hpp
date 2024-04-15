#pragma once
#include <reach/interfaces/evaluator.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <boost/signals2.hpp>

namespace benchmark {

struct ReachabilityEvaluator : public reach::Evaluator {
  double calculateScore(const std::map<std::string, double> &) const override;
};

struct ReachabilityEvaluatorFactory : public reach::EvaluatorFactory {
  reach::Evaluator::ConstPtr create(const YAML::Node &) const override;
};

}  // namespace benchmark