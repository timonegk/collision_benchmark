#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

std::string get_xacro(const std::string &xacro_file);

void declare_and_set_parameter(const rclcpp::Node::SharedPtr &node, const rclcpp::Parameter &parameter);