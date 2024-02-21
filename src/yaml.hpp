#pragma once

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <yaml-cpp/yaml.h>


namespace YAML {
template<>
struct convert<moveit_msgs::msg::RobotState> {
    static bool decode(const Node& node, moveit_msgs::msg::RobotState& state);
};

template<>
struct convert<geometry_msgs::msg::TransformStamped> {
    static bool decode(const Node& node, geometry_msgs::msg::TransformStamped& transform);
};

template<>
struct convert<shape_msgs::msg::SolidPrimitive> {
    static bool decode(const Node& node, shape_msgs::msg::SolidPrimitive& primitive);
};

template<>
struct convert<geometry_msgs::msg::Pose> {
    static bool decode(const Node& node, geometry_msgs::msg::Pose& pose);
};

template<>
struct convert<moveit_msgs::msg::CollisionObject> {
    static bool decode(const Node& node, moveit_msgs::msg::CollisionObject& object);
};

template<>
struct convert<moveit_msgs::msg::AllowedCollisionEntry> {
    static bool decode(const Node& node, moveit_msgs::msg::AllowedCollisionEntry& entry);
};

template<>
struct convert<moveit_msgs::msg::AllowedCollisionMatrix> {
    static bool decode(const Node& node, moveit_msgs::msg::AllowedCollisionMatrix& matrix);
};

template<>
struct convert<moveit_msgs::msg::PlanningScene> {
    static bool decode(const Node& node, moveit_msgs::msg::PlanningScene& scene);
};

template<>
struct convert<moveit_msgs::msg::JointConstraint> {
    static bool decode(const Node& node, moveit_msgs::msg::JointConstraint& constraint);
};
}