#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shapes.h>

std::string get_xacro(const std::string &xacro_file);

void declare_and_set_parameter(const rclcpp::Node::SharedPtr &node, const rclcpp::Parameter &parameter);

shape_msgs::msg::Mesh convertToMeshMsg(const shapes::Mesh &mesh);