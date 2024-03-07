#pragma once
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_state/conversions.h>
#include "benchmark_logger.hpp"
#include "benchmark_visualizer.hpp"

class PlanningBenchmark {
 public:
  explicit PlanningBenchmark(rclcpp::Node::SharedPtr node) : node_(std::move(node)), logger_(node_->get_logger()),
                                                             robot_model_loader_(node_),
                                                             robot_model_(robot_model_loader_.getModel()),
                                                             robot_state_(new moveit::core::RobotState(robot_model_)),
                                                             benchmarking_visualizer_(node_)
                                                             {};
  void run(const std::string &log_file);
 private:
  void initialize(const std::string& log_file);
  void gather_data();
  void setPlanningParameters();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  std::string planning_group_name_;
  moveit::core::JointModelGroup* joint_model_group_{};
  std::string tip_link_name_;
  BenchmarkVisualizer benchmarking_visualizer_;
};
