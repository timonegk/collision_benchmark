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


shape_msgs::msg::Mesh convertToMeshMsg(const shapes::Mesh &mesh) {
  shape_msgs::msg::Mesh mesh_msg;

  // Add vertices to the mesh_msg
  for (unsigned int i = 0; i < mesh.vertex_count; ++i) {
    geometry_msgs::msg::Point point;
    point.x = mesh.vertices[i * 3];
    point.y = mesh.vertices[i * 3 + 1];
    point.z = mesh.vertices[i * 3 + 2];
    mesh_msg.vertices.push_back(point);
  }

  // Add triangles to the mesh_msg
  for (unsigned int i = 0; i < mesh.triangle_count; ++i) {
    shape_msgs::msg::MeshTriangle triangle;
    triangle.vertex_indices[0] = mesh.triangles[i * 3];
    triangle.vertex_indices[1] = mesh.triangles[i * 3 + 1];
    triangle.vertex_indices[2] = mesh.triangles[i * 3 + 2];
    mesh_msg.triangles.push_back(triangle);
  }

  return mesh_msg;
}
