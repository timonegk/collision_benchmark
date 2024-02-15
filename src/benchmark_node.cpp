#include "benchmark.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>


std::string get_xacro(const std::string& xacro_file) {
  std::string xacro_command = "xacro " + xacro_file;
  std::string xacro_output;
  FILE* stream = popen(xacro_command.c_str(), "r");
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

void declare_and_set_parameter(const rclcpp::Node::SharedPtr& node, const rclcpp::Parameter& parameter) {
  node->declare_parameter(parameter.get_name(), parameter.get_type());
  node->set_parameter(parameter);
}

void set_elise_parameters(const rclcpp::Node::SharedPtr& node) {
  declare_and_set_parameter(node, rclcpp::Parameter("planning_group", "all"));
  std::string xacro_file = ament_index_cpp::get_package_share_directory("elise_description") + "/urdf/robot.urdf.xacro";
  std::string robot_description = get_xacro(xacro_file);
  std::string semantic_file = ament_index_cpp::get_package_share_directory("elise_moveit_config") + "/config/robot.srdf";
  std::ifstream file(semantic_file);
  std::string robot_description_semantic((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  declare_and_set_parameter(node, rclcpp::Parameter("robot_description", robot_description));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_semantic", robot_description_semantic));
}


void set_kinematics_config(const rclcpp::Node::SharedPtr& node) {
  //declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver", "pick_ik/PickIkPlugin"));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin"));
  //declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver", "bio_ik/BioIKKinematicsPlugin"));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_search_resolution", 0.001));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_timeout", 1.0));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.memetic_gd_max_iters", 5));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.memetic_population_size", 40));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.stop_optimization_on_valid_solution", true));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.cost_threshold", 10000.0));
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("benchmark_node");
  declare_and_set_parameter(node, rclcpp::Parameter("random_seed", 0));
  declare_and_set_parameter(node, rclcpp::Parameter("sample_size", 1000));
  declare_and_set_parameter(node, rclcpp::Parameter("ik_timeout", 0.1));
  declare_and_set_parameter(node, rclcpp::Parameter("ik_iteration_display_step", 10));

  set_elise_parameters(node);
  set_kinematics_config(node);

  IKBenchmarking ik_benchmarking(node);
  ik_benchmarking.run("kdl.csv");
  return 0;
}
