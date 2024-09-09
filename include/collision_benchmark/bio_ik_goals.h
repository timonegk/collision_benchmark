#include <bio_ik/goal.h>

namespace bio_ik {

class RCMGoal : public Goal {
  tf2::Vector3 point_;
  mutable size_t link_count_ = 0;

public:
  RCMGoal(const tf2::Vector3 &point, double weight = 1.0) {
    weight_ = weight;
    point_ = point;
  }
  void describe(GoalContext &context) const;
  double evaluate(const GoalContext &context) const;
};

class RCMGoal3 : public Goal {
  tf2::Vector3 point_;
  mutable size_t link_count_ = 0;
  std::string link_name_;

public:
  RCMGoal3(const std::string &link_name, const tf2::Vector3 &point,
           double weight = 1.0) {
    weight_ = weight;
    point_ = point;
    link_name_ = link_name;
  }
  void describe(GoalContext &context) const;
  double evaluate(const GoalContext &context) const;
};

}
