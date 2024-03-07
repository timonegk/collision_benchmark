#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>

class BenchmarkVisualizer {
 public:
  explicit BenchmarkVisualizer(rclcpp::Node::SharedPtr &node);
  void visualize_robot_state(const moveit::core::RobotStateConstPtr &robot_state) const;
  void visualize_collision_object(const moveit_msgs::msg::CollisionObject &collision_object) const;
  void visualize_constraint(const moveit_msgs::msg::PositionConstraint &constraint) const;
  void visualize_pose(const geometry_msgs::msg::PoseStamped &pose) const;
 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_object_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraint_publisher_;
  std::string planning_group_name_;
};