#include "benchmark.hpp"

#include <fmt/core.h>
#include <fmt/ranges.h>

#include <chrono>
#include <numeric>

using namespace std::chrono_literals;

void IKBenchmarking::initialize(const std::string &log_file) {
  const auto seed = static_cast<unsigned int>(node_->get_parameter("random_seed").as_int());
  random_numbers::RandomNumberGenerator generator_{seed};

  planning_group_name_ = node_->get_parameter("planning_group").as_string();
  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);

  // Load the tip link name (not the end effector)
  auto const &link_names = joint_model_group_->getLinkModelNames();

  if (!link_names.empty()) {
    tip_link_name_ = link_names.back();
  } else {
    RCLCPP_ERROR(logger_, "ERROR: The move group is corrupted. Links count is zero.\n");
    rclcpp::shutdown();
  }

  robot_state_->setToDefaultValues();

  benchmarking_logger_ = std::make_shared<BenchmarkLogger>(log_file);
  benchmarking_visualizer_ = std::make_shared<BenchmarkVisualizer>(node_);
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

}

void IKBenchmarking::gather_data() {
  // Collect IK solving data
  sample_size_ = static_cast<size_t>(node_->get_parameter("sample_size").as_int());
  ik_timeout_ = node_->get_parameter("ik_timeout").as_double();
  ik_iteration_display_step_ =
      static_cast<size_t>(node_->get_parameter("ik_iteration_display_step").as_int());
  bool follow_line = true;
  auto ik_initialization = IKBenchmarking::IKInitializationType::PREVIOUS;

  for (size_t i = 0; i < sample_size_; ++i) {
    if ((i + 1) % ik_iteration_display_step_ == 0) {
      RCLCPP_INFO(logger_, "Solved sample %ld/%ld ...", i + 1, sample_size_);
    }

    Eigen::Isometry3d tip_link_pose;
    if (!follow_line) {
      // save previous joint values
      std::vector<double> previous_joint_values;
      robot_state_->copyJointGroupPositions(joint_model_group_, previous_joint_values);

      // Generate a random state and solve Forward Kinematics (FK)
      robot_state_->setToRandomPositions(joint_model_group_, generator_);
      robot_state_->updateLinkTransforms();
      tip_link_pose = robot_state_->getGlobalLinkTransform(tip_link_name_);

      //  Log the sampled random joint values for debugging
      std::vector<double> random_joint_values;
      robot_state_->copyJointGroupPositions(joint_model_group_, random_joint_values);
      std::stringstream ss;
      ss << "[";
      for (size_t i = 0; i < random_joint_values.size(); ++i) {
        ss << random_joint_values[i];
        if (i != random_joint_values.size() - 1) {
          ss << ", ";
        }
      }
      ss << "]";
      RCLCPP_DEBUG(logger_, "The sampled random joint values are:\n%s\n", ss.str().c_str());
      // restore previous joint values
      robot_state_->setJointGroupPositions(joint_model_group_, previous_joint_values);
      robot_state_->updateLinkTransforms();
    } else {
      tip_link_pose = [&] {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        double x_value = double(i) / double(sample_size_) - 0.5;
        pose.translation().x() = x_value;
        pose.translation().y() = 0.5;
        pose.translation().z() = 1.0;
        return pose;
      }();
    }

    if (ik_initialization == IKInitializationType::RANDOM) {
      // Randomize the initial seed state of the robot before solving IK
      robot_state_->setToRandomPositions(joint_model_group_, generator_);
      robot_state_->updateLinkTransforms();
    } else if (ik_initialization == IKInitializationType::PREVIOUS) {
      // Use the previous state as the initial seed state for solving IK
      robot_state_->updateLinkTransforms();
    } else {
      // Set the robot state to the default state
      robot_state_->setToDefaultValues();
    }

    const moveit::core::GroupStateValidityCallbackFn is_valid = [&](moveit::core::RobotState *state,
                                                                    const moveit::core::JointModelGroup *group,
                                                                    const double *joint_group_variable_values) {
      const collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;
      state->setJointGroupPositions(group, joint_group_variable_values);
      state->update();
      planning_scene_->checkSelfCollision(req, res, *state);
      bool collision_free = !res.collision;
      bool joint_valid = -0.1 < joint_group_variable_values[0] && joint_group_variable_values[0] < 0.1;
      //std::cout << "collision_free: " << collision_free << " joint_valid: " << joint_valid << std::endl;
      return collision_free && joint_valid;
    };

    // Solve Inverse kinematics (IK)
    const auto start_time = std::chrono::high_resolution_clock::now();
    const bool found_ik =
        robot_state_->setFromIK(joint_model_group_, tip_link_pose, ik_timeout_, is_valid);
    const auto end_time = std::chrono::high_resolution_clock::now();

    const auto solve_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);


    // Calculate position error
    Eigen::Isometry3d ik_tip_link_pose = robot_state_->getGlobalLinkTransform(tip_link_name_);
    Eigen::Vector3d position_diff =
        ik_tip_link_pose.translation() - tip_link_pose.translation();
    double position_error = position_diff.norm();

    // Calculate orientation error (angle between two quaternions)
    Eigen::Quaterniond orientation(tip_link_pose.rotation());
    Eigen::Quaterniond ik_orientation(ik_tip_link_pose.rotation());
    double orientation_error = orientation.angularDistance(ik_orientation);

    benchmarking_logger_->log_data(i + 1, found_ik, solve_time.count(), position_error,
                                   orientation_error);
    benchmarking_visualizer_->visualize_robot_state(robot_state_);
    std::this_thread::sleep_for(10ms);
  }

  // Average IK solving time and success rate
  double average_solve_time = benchmarking_logger_->get_average_solve_time();
  double success_rate = benchmarking_logger_->get_success_rate();

  RCLCPP_INFO(logger_, "Success rate = %f and average IK solving time is %f microseconds\n",
              success_rate, average_solve_time);

  calculation_done_ = true;
}

void IKBenchmarking::run(const std::string &log_file) {
  this->initialize(log_file);
  this->gather_data();
}

double IKBenchmarking::get_success_rate() const { return benchmarking_logger_->get_success_rate(); }

double IKBenchmarking::get_average_solve_time() const { return benchmarking_logger_->get_average_solve_time(); }

bool IKBenchmarking::calculation_done() const { return calculation_done_; }
