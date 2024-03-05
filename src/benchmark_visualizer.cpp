#include "benchmark_visualizer.hpp"

BenchmarkVisualizer::BenchmarkVisualizer(rclcpp::Node::SharedPtr &node) : node_(node) {
  planning_group_name_ = node_->get_parameter("planning_group").as_string();
  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
}

void BenchmarkVisualizer::visualize_ik(const moveit::core::RobotStateConstPtr &robot_state) const {
  const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(planning_group_name_);
  const sensor_msgs::msg::JointState joint_state_msg = [&] {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = joint_model_group->getVariableNames();
    for (const std::string &name : joint_state_msg.name) {
      msg.position.push_back(robot_state->getVariablePosition(name));
    }
    return msg;
  }();
  joint_state_publisher_->publish(joint_state_msg);

}