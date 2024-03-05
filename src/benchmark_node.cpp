#include "benchmark.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <std_msgs/msg/string.hpp>

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

void set_elise_parameters(const rclcpp::Node::SharedPtr &node,
                          const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &robot_description_publisher) {
  declare_and_set_parameter(node, rclcpp::Parameter("planning_group", "all"));
  std::string xacro_file = ament_index_cpp::get_package_share_directory("elise_description") + "/urdf/robot.urdf.xacro";
  std::string robot_description = get_xacro(xacro_file);
  std::string
      semantic_file = ament_index_cpp::get_package_share_directory("elise_moveit_config") + "/config/robot.srdf";
  std::ifstream file(semantic_file);
  std::string robot_description_semantic((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  declare_and_set_parameter(node, rclcpp::Parameter("robot_description", robot_description));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_semantic", robot_description_semantic));
  robot_description_publisher->publish([&] {
    auto msg = std_msgs::msg::String();
    msg.data = robot_description;
    return msg;
  }());
}

void set_kinematics_config_kdl(const rclcpp::Node::SharedPtr &node) {
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver",
                                              "kdl_kinematics_plugin/KDLKinematicsPlugin"));
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_search_resolution",
                                              0.001));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_timeout", 1.0));
}

void set_kinematics_config_trac_ik(const rclcpp::Node::SharedPtr &node, const std::string &solve_type) {
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver",
                                              "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin"));
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_search_resolution",
                                              0.001));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_timeout", 1.0));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.solve_type",
                                                    solve_type));
}

void set_kinematics_config_pick_ik(const rclcpp::Node::SharedPtr &node) {
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver",
                                              "pick_ik/PickIkPlugin"));

  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_search_resolution",
                                              0.001));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_timeout", 1.0));
  /*declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.memetic_gd_max_iters", 5));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.memetic_population_size", 40));
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.stop_optimization_on_valid_solution",
                                              true));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.cost_threshold", 10000.0));
  */
}

void set_kinematics_config_bio_ik(const rclcpp::Node::SharedPtr &node) {
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver",
                                              "bio_ik/BioIKKinematicsPlugin"));
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_search_resolution",
                                              0.001));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.kinematics_solver_timeout", 1.0));
  /*declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.memetic_gd_max_iters", 5));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.memetic_population_size", 40));
  declare_and_set_parameter(node,
                            rclcpp::Parameter("robot_description_kinematics.all.stop_optimization_on_valid_solution",
                                              true));
  declare_and_set_parameter(node, rclcpp::Parameter("robot_description_kinematics.all.cost_threshold", 10000.0));
  */
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("benchmark_node");
  auto robot_description_publisher = node->create_publisher<std_msgs::msg::String>("robot_description", rclcpp::QoS(1)
      .transient_local());
  declare_and_set_parameter(node, rclcpp::Parameter("random_seed", 0));
  declare_and_set_parameter(node, rclcpp::Parameter("sample_size", 1000));
  declare_and_set_parameter(node, rclcpp::Parameter("ik_timeout", 0.05));
  declare_and_set_parameter(node, rclcpp::Parameter("ik_iteration_display_step", 10));

  set_elise_parameters(node, robot_description_publisher);
  std::filesystem::path pkg_share_dir(ament_index_cpp::get_package_share_directory("benchmark"));
  std::filesystem::path results_dir(pkg_share_dir / "results");
  std::filesystem::create_directories(results_dir);
  /*{
    set_kinematics_config_kdl(node);
    IKBenchmarking ik_benchmarking(node);
    ik_benchmarking.run((results_dir / "kdl.csv").string());
  }
  {
    set_kinematics_config_bio_ik(node);
    IKBenchmarking ik_benchmarking(node);
    ik_benchmarking.run((results_dir / "bio_ik.csv").string());
  }*/
  {
    set_kinematics_config_pick_ik(node);
    IKBenchmarking ik_benchmarking(node);
    ik_benchmarking.run((results_dir / "pick_ik.csv").string());
  }
  /*{
    set_kinematics_config_trac_ik(node, "Speed");
    IKBenchmarking ik_benchmarking(node);
    ik_benchmarking.run((results_dir / "trac_ik_speed.csv").string());
  }
  {
    set_kinematics_config_trac_ik(node, "Distance");
    IKBenchmarking ik_benchmarking(node);
    ik_benchmarking.run((results_dir / "trac_ik_distance.csv").string());
  }*/
  //return std::system("ros2 run benchmark plot_data.py");
  return 0;
}
