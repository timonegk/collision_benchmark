#include <collision_benchmark/bio_ik_goals.h>

namespace bio_ik {


  void RCMGoal::describe(GoalContext &context) const {
    Goal::describe(context);
    link_count_ = 0;
    for (auto &link_name : context.getRobotModel().getLinkModelNames()) {
      context.addLink(link_name);
      link_count_++;
    }
  }
  double RCMGoal::evaluate(const GoalContext &context) const {
    double min_distance = FLT_MAX;
    for (size_t i = 0; i < link_count_ - 1; ++i) {
      auto &frame = context.getLinkFrame(i);
      auto &next_frame = context.getLinkFrame(i + 1);
      tf2::Vector3 projected_point;
      if (frame.pos != next_frame.pos) {
        double t = (point_ - frame.pos).dot(next_frame.pos - frame.pos) /
                   next_frame.pos.distance2(frame.pos);
        double clipped_t = std::clamp(t, 0.0, 1.0);
        projected_point = frame.pos + clipped_t * (next_frame.pos - frame.pos);
      } else {
        projected_point = frame.pos;
      }
      double distance = point_.distance2(projected_point);
      min_distance = std::min(min_distance, distance);
    }
    return min_distance;
  }


  void RCMGoal3::describe(GoalContext &context) const {
    Goal::describe(context);
    bool added = false;
    for (auto &link_name : context.getRobotModel().getLinkModelNames()) {
      if (link_name == link_name_) {
        context.addLink(link_name);
        added = true;
      } else if (added) {
        // also add next
        context.addLink(link_name);
        break;
      }
    }
  }
  double RCMGoal3::evaluate(const GoalContext &context) const {
    auto &frame = context.getLinkFrame(0);
    auto &next_frame = context.getLinkFrame(1);
    tf2::Vector3 projected_point;
    if (frame.pos != next_frame.pos) {
      double t = (point_ - frame.pos).dot(next_frame.pos - frame.pos) /
                 next_frame.pos.distance2(frame.pos);
      double clipped_t = std::clamp(t, 0.0, 1.0);
      projected_point = frame.pos + clipped_t * (next_frame.pos - frame.pos);
    } else {
      projected_point = frame.pos;
    }
    double distance = point_.distance2(projected_point);
    return distance;
  }

}
