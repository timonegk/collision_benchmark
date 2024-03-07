#include "utils.hpp"
#include <fstream>

std::string get_xacro(const std::string &xacro_file) {
  std::string xacro_command = "xacro " + xacro_file;
  std::string xacro_output;
  FILE *stream = popen(xacro_command.c_str(), "r");
  if (stream) {
    const int max_buffer = 256;
    char buffer[max_buffer];
    while (!feof(stream)) {
      if (fgets(buffer, max_buffer, stream) != nullptr) {
        xacro_output.append(buffer);
      }
    }
    pclose(stream);
  }
  return xacro_output;
}

void declare_and_set_parameter(const rclcpp::Node::SharedPtr &node, const rclcpp::Parameter &parameter) {
  if (!node->has_parameter(parameter.get_name())) {
    node->declare_parameter(parameter.get_name(), parameter.get_type());
  }
  node->set_parameter(parameter);
}