#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>

class BenchmarkVisualizer {
 public:
  explicit BenchmarkVisualizer(rclcpp::Node::SharedPtr &node);
  void visualize_ik(const moveit::core::RobotStateConstPtr &robot_state) const;
 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::string planning_group_name_;
};