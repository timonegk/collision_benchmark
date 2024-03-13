#include "benchmark_visualizer.hpp"
#include <rviz_rendering/mesh_loader.hpp>

BenchmarkVisualizer::BenchmarkVisualizer(rclcpp::Node::SharedPtr &node) : node_(node) {
  planning_group_name_ = node_->get_parameter("planning_group").as_string();
  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
  collision_object_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("collision_object", 10);
  constraint_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("constraint", 10);
}

void BenchmarkVisualizer::visualize_robot_state(const moveit::core::RobotStateConstPtr &robot_state) const {
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

void BenchmarkVisualizer::visualize_collision_object(const moveit_msgs::msg::CollisionObject &collision_object) const {
  visualization_msgs::msg::Marker marker;
  marker.header = collision_object.header;
  marker.id = 0;
  if (!collision_object.primitives.empty()) {
    if (collision_object.primitives[0].type == shape_msgs::msg::SolidPrimitive::BOX) {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = collision_object.primitives[0].dimensions[0];
      marker.scale.y = collision_object.primitives[0].dimensions[1];
      marker.scale.z = collision_object.primitives[0].dimensions[2];
    } else if (collision_object.primitives[0].type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.scale.x = collision_object.primitives[0].dimensions[0];
      marker.scale.y = collision_object.primitives[0].dimensions[0];
      marker.scale.z = collision_object.primitives[0].dimensions[1];
    }
  } else if (!collision_object.meshes.empty()) {
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = collision_object.id;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
  }
  marker.ns = "visualization";
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = collision_object.pose;
  marker.color.r = 0.9;
  marker.color.g = 0.9;
  marker.color.b = 0.9;
  marker.color.a = 1.0;
  collision_object_publisher_->publish(marker);
}

void BenchmarkVisualizer::visualize_constraint(const moveit_msgs::msg::PositionConstraint &constraint) const {
  const visualization_msgs::msg::MarkerArray marker = [&]{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = constraint.header.frame_id;
    marker.header.stamp = node_->now();
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = constraint.constraint_region.primitive_poses[0];
    marker.scale.x = constraint.constraint_region.primitives[0].dimensions[0];
    marker.scale.y = constraint.constraint_region.primitives[0].dimensions[1];
    marker.scale.z = constraint.constraint_region.primitives[0].dimensions[2];
    marker.color.g = 1.0;
    marker.color.a = 0.5;
    marker_array.markers.push_back(marker);
    return marker_array;
  }();
  constraint_publisher_->publish(marker);
}

void BenchmarkVisualizer::visualize_pose(const geometry_msgs::msg::PoseStamped &pose) const {
  pose_publisher_->publish(pose);
}
















