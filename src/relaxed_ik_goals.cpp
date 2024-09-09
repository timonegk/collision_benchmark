#include <collision_benchmark/relaxed_ik_goals.h>

namespace relaxed_ik {

static double groove_loss(double x, double t, double d, double c, double f,
                          double g) {
  return -exp((-pow(x - t, d)) / (2.0 * pow(c, 2))) + f * pow(x - t, g);
}

static double swamp_loss(double x, double l_bound, double u_bound, double f1,
                         double f2, double n) {
  double x_scaled = (2 * x - l_bound - u_bound) / (u_bound - l_bound);
  double b = std::pow((-1 / std::log(0.05)), 1 / n);
  return (f1 + f2 * std::pow(x_scaled, 2)) *
             (1 - std::exp(-std::pow(x_scaled / b, n))) -
         1.0;
}

double EnvCollisionDistance::call(const std::vector<double> &, const Variables &v,
                                  const moveit::core::RobotState &state) {
    // for o in objects
    //  for shape in objects.shapes
    //   add shape as collision object
    //   apply shape's global transform to collision object
    // this only has to be once at initialization because the objects don't change
    // later, only do distance checks from links to objects
    // also evaluate whether single-link distances are better than one total distance
    // why would they be? maybe better gradients? bc small movements might produce smaller changes
    // but I'm not sure if this makes sense
    // all-distance = min(distances)
    // for example it changes how big the gradient is:
    // if only one link gets closer, the gradient is small, if all get closer, it is bigger
    // with all-distance, this is not the case, the gradient is independent of how many links move

    collision_detection::AllowedCollisionMatrix acm;
    acm.setDefaultEntry("end_effector", true);
    double distance = planning_scene_->distanceToCollision(state, acm);
    //double penalty_cutoff = 0.02;
    double penalty_cutoff = 0.01;
    // distance cost is 1 if distance == penalty_cutoff
    double distance_cost = std::pow(2 * penalty_cutoff / (distance + penalty_cutoff), 2);
    // with this groove loss, the loss is always -1 unless we get close to zero
    // still, a value of 1 is relatively good, we might need to decrease c to get better values
    return groove_loss(distance_cost, 0, 2, 2.5, 0.0035, 4);
}

double EnvCollisionDistance2::call(const std::vector<double> &, const relaxed_ik::Variables &v,
                                  const moveit::core::RobotState &state) {
    collision_detection::AllowedCollisionMatrix acm;
    acm.setDefaultEntry("end_effector", true);
    acm.setDefaultEntry("base_link_inertia", true);
    acm.setDefaultEntry("shoulder_link", true);
    acm.setDefaultEntry("upper_arm_link", true);
    acm.setDefaultEntry("forearm_link", true);
    acm.setDefaultEntry("wrist_1_link", true);
    acm.setDefaultEntry("wrist_2_link", true);
    acm.setDefaultEntry("wrist_3_link", true);
    double distance = planning_scene_->distanceToCollision(state, acm);
    //double penalty_cutoff = 0.02;
    double penalty_cutoff = 0.01;
    // distance cost is 1 if distance == penalty_cutoff
    double distance_cost = std::pow(2 * penalty_cutoff / (distance + penalty_cutoff), 2);
    // with this groove loss, the loss is always -1 unless we get close to zero
    // still, a value of 1 is relatively good, we might need to decrease c to get better values
    return groove_loss(distance_cost, 0, 2, 2.5, 0.0035, 4);
}


double EnvCollisionDepth::call(const std::vector<double> &, const relaxed_ik::Variables &v,
                          const moveit::core::RobotState &state) {
    collision_detection::CollisionRequest req;
    req.contacts = true;
    collision_detection::CollisionResult res;
    planning_scene_->checkCollision(req, res, state);
    double penetration_depth = 0;
    for (const auto &[link_names, contacts] : res.contacts) {
        for (const auto &contact : contacts) {
            penetration_depth += contact.depth;
        }
    }
    double cost = std::pow(penetration_depth / 0.05, 2);
    return groove_loss(cost, 0, 2, 0.1, 0.0035, 2);
}

double EnvCollisionDepth2::call(const std::vector<double> &, const relaxed_ik::Variables &v,
                          const moveit::core::RobotState &state) {
    collision_detection::AllowedCollisionMatrix acm;
    acm.setDefaultEntry("base_link_inertia", true);
    acm.setDefaultEntry("shoulder_link", true);
    acm.setDefaultEntry("upper_arm_link", true);
    acm.setDefaultEntry("forearm_link", true);
    acm.setDefaultEntry("wrist_1_link", true);
    acm.setDefaultEntry("wrist_2_link", true);
    acm.setDefaultEntry("wrist_3_link", true);
    acm.setEntry("endo_box", "endo_first_link", true);
    acm.setEntry("endo_first_link", "endo_second_link", true);
    acm.setEntry("endo_second_link", "endo_third_link", true);
    collision_detection::CollisionRequest req;
    req.contacts = true;
    collision_detection::CollisionResult res;
    planning_scene_->checkCollision(req, res, state, acm);
    double penetration_depth = 0;
    for (const auto &[link_names, contacts] : res.contacts) {
        for (const auto &contact : contacts) {
            penetration_depth += contact.depth;
        }
    }
    double cost = std::pow(penetration_depth / 0.05, 2);
    return groove_loss(cost, 0, 2, 0.1, 0.0035, 2);
}

double RCMGoal::call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) {
    double min_distance = FLT_MAX;
    for (std::size_t i = 0; i < state.getRobotModel()->getLinkModelCount() - 1; ++i) {
        const moveit::core::LinkModel *frame_model = state.getRobotModel()->getLinkModel(i);
        const moveit::core::LinkModel *next_frame_model = state.getRobotModel()->getLinkModel(i + 1);
        const Eigen::Isometry3d frame = state.getGlobalLinkTransform(frame_model);
        const Eigen::Isometry3d next_frame = state.getGlobalLinkTransform(next_frame_model);
        Eigen::Vector3d projected_point;
        if (frame.translation() != next_frame.translation()) {
            const double t = (point_ - frame.translation()).dot(next_frame.translation() - frame.translation()) / (frame.translation() - next_frame.translation()).squaredNorm();
            const double clipped_t = std::clamp(t, 0.0, 1.0);
            projected_point = frame.translation() + clipped_t * (next_frame.translation() - frame.translation());
        } else {
            projected_point = frame.translation();
        }
        const double distance = (point_ - projected_point).squaredNorm();
        min_distance = std::min(min_distance, distance);
    }
    return groove_loss(min_distance, 0, 2, 0.01, 10, 2);
}

RCMGoal2::RCMGoal2(const moveit::core::RobotModelConstPtr &robot_model, const Eigen::Vector3d &point) : planning_scene_(robot_model) {
    moveit_msgs::msg::CollisionObject co;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions.push_back(0.01);
    co.primitives.push_back(primitive);
    co.primitive_poses.emplace_back();
    co.primitive_poses[0].position.x = point.x();
    co.primitive_poses[0].position.y = point.y();
    co.primitive_poses[0].position.z = point.z();
    co.operation = moveit_msgs::msg::CollisionObject::ADD;
    co.header.frame_id = "world";
    co.id = "RCM";
    planning_scene_.processCollisionObjectMsg(co);
    acm_.setDefaultEntry("RCM", true);
    acm_.setEntry("endo_first_link", "RCM", false);
}

double RCMGoal2::call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) {
    double distance = planning_scene_.distanceToCollision(state, acm_);
    distance = std::max(distance, 0.0);
    return groove_loss(distance, 0, 2, 0.01, 10, 2);
}

double RCMGoal3::call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) {
    std::size_t link_index = 0;
    for (std::size_t i = 0; i < state.getRobotModel()->getLinkModelCount() - 1; ++i) {
        if (state.getRobotModel()->getLinkModelNames()[i] == link_name_) {
            link_index = i;
            break;
        }
    }
    const moveit::core::LinkModel *frame_model = state.getRobotModel()->getLinkModel(link_index);
    const moveit::core::LinkModel *next_frame_model = state.getRobotModel()->getLinkModel(link_index + 1);
    const Eigen::Isometry3d frame = state.getGlobalLinkTransform(frame_model);
    const Eigen::Isometry3d next_frame = state.getGlobalLinkTransform(next_frame_model);
    Eigen::Vector3d projected_point;
    if (frame.translation() != next_frame.translation()) {
        const double t = (point_ - frame.translation()).dot(next_frame.translation() - frame.translation()) / (frame.translation() - next_frame.translation()).squaredNorm();
        const double clipped_t = std::clamp(t, 0.0, 1.0);
        projected_point = frame.translation() + clipped_t * (next_frame.translation() - frame.translation());
    } else {
        projected_point = frame.translation();
    }
    const double distance = (point_ - projected_point).squaredNorm();
    return groove_loss(distance, 0, 2, 0.01, 10, 2);
}

double LineGoal::call(const std::vector<double> &joints, const Variables &v,
                      const moveit::core::RobotState &state) {
    Eigen::Vector3d link_position = state.getGlobalLinkTransform(link_name_).translation();
    const Eigen::Vector3d p = link_position - axis_ * axis_.dot(link_position - point_);
    double dist = (point_ - p).squaredNorm();
    return groove_loss(dist, 0, 2, 0.1, 10, 2);
}

double AlignmentGoal::call(const std::vector<double> &joints, const Variables &v,
                           const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& link_pose = state.getGlobalLinkTransform(link_name_);
    const Eigen::Vector3d axis = link_pose.rotation() * Eigen::Vector3d(0, 1, 0);
    double dist = (axis - axis_).squaredNorm();
    return groove_loss(dist, 0, 2, 0.1, 10, 2);
}

double ScanGoal::call(const std::vector<double> &joints, const Variables &v,
                           const moveit::core::RobotState &state) {
    const Eigen::Isometry3d& current = state.getGlobalLinkTransform(v.ee_name);
    const Eigen::Isometry3d& target = v.target_pose;
    const Eigen::Vector3d current_to_target = target.translation() - current.translation();
    const double distance = current_to_target.norm();

    const Eigen::Vector3d target_normal = target.rotation() * Eigen::Vector3d{0, 0, 1};
    const double normal_angle = (target_normal - current_to_target.normalized()).norm();

    const Eigen::Vector3d sensor_normal = current.rotation() * Eigen::Vector3d{0, 0, 1};
    const double fov_angle = (sensor_normal - current_to_target.normalized()).norm();

    return (swamp_loss(distance, 0, 0.02, 1.0, 0.1, 8) +
            swamp_loss(normal_angle, -0.1, 0.1, 1.0, 0.1, 8) +
            swamp_loss(fov_angle, -0.1, 0.1, 1.0, 0.1, 8));
}

}
