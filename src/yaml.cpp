#include "yaml.hpp"
#include <iostream>
#include <string>

namespace YAML {
bool convert<moveit_msgs::msg::RobotState>::decode(
    const Node &node,
    moveit_msgs::msg::RobotState &state) {
  state.joint_state.name = node["joint_state"]["name"].as<std::vector<std::string>>();
  state.joint_state.position = node["joint_state"]["position"].as<std::vector<double>>();
  return true;
}

bool convert<geometry_msgs::msg::TransformStamped>::decode(
    const Node &node,
    geometry_msgs::msg::TransformStamped &transform) {
  if (node["header"]) {
    transform.header.frame_id = node["header"]["frame_id"].as<std::string>();
    transform.header.stamp.sec = node["header"]["stamp"]["sec"].as<int>();
    transform.header.stamp.nanosec = node["header"]["stamp"]["nanosec"].as<int>();
  }
  transform.child_frame_id = node["child_frame_id"].as<std::string>();
  transform.transform.translation.x = node["transform"]["translation"][0].as<double>();
  transform.transform.translation.y = node["transform"]["translation"][1].as<double>();
  transform.transform.translation.z = node["transform"]["translation"][2].as<double>();
  transform.transform.rotation.x = node["transform"]["rotation"][0].as<double>();
  transform.transform.rotation.y = node["transform"]["rotation"][1].as<double>();
  transform.transform.rotation.z = node["transform"]["rotation"][2].as<double>();
  transform.transform.rotation.w = node["transform"]["rotation"][3].as<double>();
  return true;
}

bool convert<shape_msgs::msg::SolidPrimitive>::decode(
    const Node &node,
    shape_msgs::msg::SolidPrimitive &primitive) {
  auto type = node["type"].as<std::string>();
  if (type == "box") {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  } else if (type == "sphere") {
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  } else if (type == "cylinder") {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  } else if (type == "cone") {
    primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
  } else if (type == "prism") {
    primitive.type = shape_msgs::msg::SolidPrimitive::PRISM;
  } else {
    return false;
  }
  for (YAML::const_iterator it = node["dimensions"].begin(); it != node["dimensions"].end();
       ++it) {
    primitive.dimensions.push_back(it->as<double>());
  }
  return true;
}

bool convert<geometry_msgs::msg::Pose>::decode(const Node &node, geometry_msgs::msg::Pose &pose) {
  pose.position.x = node["position"][0].as<double>();
  pose.position.y = node["position"][1].as<double>();
  pose.position.z = node["position"][2].as<double>();
  pose.orientation.x = node["orientation"][0].as<double>();
  pose.orientation.y = node["orientation"][1].as<double>();
  pose.orientation.z = node["orientation"][2].as<double>();
  pose.orientation.w = node["orientation"][3].as<double>();
  return true;
}

bool convert<moveit_msgs::msg::CollisionObject>::decode(
    const Node &node,
    moveit_msgs::msg::CollisionObject &object) {
  object.id = node["id"].as<std::string>();
  object.primitives = node["primitives"].as<std::vector<shape_msgs::msg::SolidPrimitive>>();
  object.primitive_poses = node["primitive_poses"].as<std::vector<geometry_msgs::msg::Pose>>();
  return true;
}

bool convert<moveit_msgs::msg::AllowedCollisionEntry>::decode(
    const Node &node,
    moveit_msgs::msg::AllowedCollisionEntry &entry) {
  entry.enabled = node.as<std::vector<bool>>();
  return true;
}
bool convert<moveit_msgs::msg::AllowedCollisionMatrix>::decode(
    const Node &node,
    moveit_msgs::msg::AllowedCollisionMatrix &matrix) {
  matrix.entry_names = node["entry_names"].as<std::vector<std::string>>();
  matrix.entry_values =
      node["entry_values"].as<std::vector<moveit_msgs::msg::AllowedCollisionEntry>>();
  return true;
}

bool convert<moveit_msgs::msg::PlanningScene>::decode(
    const Node &node,
    moveit_msgs::msg::PlanningScene &scene) {
  scene.robot_model_name = node["robot_model_name"].as<std::string>();
  scene.robot_state = node["robot_state"].as<moveit_msgs::msg::RobotState>();
  scene.name = node["name"].as<std::string>();
  scene.fixed_frame_transforms =
      node["fixed_frame_transforms"].as<std::vector<geometry_msgs::msg::TransformStamped>>();
  scene.allowed_collision_matrix =
      node["allowed_collision_matrix"].as<moveit_msgs::msg::AllowedCollisionMatrix>();
  scene.world.collision_objects =
      node["world"]["collision_objects"].as<std::vector<moveit_msgs::msg::CollisionObject>>();
  return true;
}

bool convert<moveit_msgs::msg::JointConstraint>::decode(
    const Node &node,
    moveit_msgs::msg::JointConstraint &constraint) {
    constraint.joint_name = node["joint_name"].as<std::string>();
    constraint.position = node["position"].as<double>();
    return true;
};
}
