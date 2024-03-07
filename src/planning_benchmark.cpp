#include "planning_benchmark.hpp"
#include "utils.hpp"
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>

using namespace std::chrono_literals;

void PlanningBenchmark::run(const std::string &log_file) {
  RCLCPP_INFO(node_->get_logger(), "Running Planning Benchmark");
  initialize(log_file);
  gather_data();
}

void PlanningBenchmark::setPlanningParameters() {
  declare_and_set_parameter(node_, rclcpp::Parameter("ompl.planning_plugins",
                                                     std::vector<std::string>{"ompl_interface/OMPLPlanner"}));
  declare_and_set_parameter(node_, rclcpp::Parameter("ompl.request_adapters",
                                                     std::vector<std::string>{
                                                         "default_planning_request_adapters/ResolveConstraintFrames",
                                                         "default_planning_request_adapters/ValidateWorkspaceBounds",
                                                         "default_planning_request_adapters/CheckStartStateBounds",
                                                         "default_planning_request_adapters/CheckStartStateCollision"}));
  declare_and_set_parameter(node_, rclcpp::Parameter("ompl.response_adapters",
                                                     std::vector<std::string>{
                                                         //"default_planning_response_adapters/AddTimeOptimalParameterization",
                                                         "default_planning_response_adapters/ValidateSolution",
                                                         "default_planning_response_adapters/DisplayMotionPath"}));
  declare_and_set_parameter(node_, rclcpp::Parameter("ompl.planner_configs.RRTConnect.type", "geometric::RRTConnect"));
  declare_and_set_parameter(node_, rclcpp::Parameter("ompl.planner_configs.RRTConnect.range", 0.0));
  declare_and_set_parameter(node_, rclcpp::Parameter(
      std::string("ompl.") + planning_group_name_ + ".projection_evaluator",
      "joints(shoulder_pan_joint,shoulder_lift_joint)"));
}

void PlanningBenchmark::initialize(const std::string &log_file) {
  planning_group_name_ = node_->get_parameter("planning_group").as_string();
  setPlanningParameters();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, node_, "ompl");
  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
  tip_link_name_ = joint_model_group_->getLinkModelNames().back();
}

void PlanningBenchmark::gather_data() {
  //planning_scene_->processCollisionObjectMsg()
  const geometry_msgs::msg::PoseStamped pose = [] {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 0.9;
    pose.pose.position.z = 1.0;
    pose.pose.orientation.w = 0.707;
    pose.pose.orientation.z = 0.707;
    return pose;
  }();
  benchmarking_visualizer_.visualize_pose(pose);
  const moveit_msgs::msg::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(tip_link_name_,
                                                                                                  pose);
  robot_state_->setToDefaultValues();
  robot_state_->update();
  benchmarking_visualizer_.visualize_robot_state(robot_state_);
  const moveit_msgs::msg::CollisionObject collision_box = [] {
    moveit_msgs::msg::CollisionObject obj;
    obj.pose.position.x = 1.0;
    obj.pose.position.y = 0.5;
    obj.pose.position.z = 0.5;
    obj.header.frame_id = "base_link";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.5, 0.5, 0.1};
    obj.primitives.push_back(box);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;
    return obj;
  }();
  if (!planning_scene_->processCollisionObjectMsg(collision_box)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not process collision object");
    return;
  }
  benchmarking_visualizer_.visualize_collision_object(collision_box);

  const moveit_msgs::msg::PositionConstraint box_constraint = [&] {
    moveit_msgs::msg::PositionConstraint box_constraint;
    box_constraint.header.frame_id = "base_link";
    box_constraint.link_name = tip_link_name_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {2.0, 2.0, 1.2};
    box_constraint.constraint_region.primitives.push_back(box);
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 1.0;
    box_pose.position.y = 0.5;
    box_pose.position.z = 0.6;
    box_pose.orientation.w = 1.0;
    box_constraint.constraint_region.primitive_poses.push_back(box_pose);
    box_constraint.weight = 1.0;
    return box_constraint;
  }();
  benchmarking_visualizer_.visualize_constraint(box_constraint);
  const planning_interface::MotionPlanRequest req = [&] {
    planning_interface::MotionPlanRequest req;
    req.pipeline_id = "ompl";
    req.planner_id = "RRTConnect";
    req.group_name = "all";
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 10.0;
    moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);
    req.goal_constraints.push_back(pose_goal);
    req.path_constraints.position_constraints.push_back(box_constraint);
    return req;
  }();
  planning_interface::MotionPlanResponse res;
  if (!planning_pipeline_->generatePlan(planning_scene_, req, res)
      || res.error_code.val != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR_STREAM(logger_, "Could not compute plan successfully, error is " << res.error_code.val);
  } else {
    RCLCPP_INFO(logger_, "Plan computed successfully");
    RCLCPP_INFO_STREAM(logger_, "Plan duration: " << res.trajectory->getDuration());
    RCLCPP_INFO_STREAM(logger_, "Plan length: " << res.trajectory->getWayPointCount());
    std::for_each(res.trajectory->begin(), res.trajectory->end(), [&](const std::pair<moveit::core::RobotStatePtr, double> &waypoint) {
      benchmarking_visualizer_.visualize_robot_state(waypoint.first);
      std::this_thread::sleep_for(100ms);
    });
  }
}