#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <yaml-cpp/yaml.h>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include "yaml.hpp"

moveit_msgs::msg::PlanningScene load_scene(const std::string &file) {
  YAML::Node config_file = YAML::LoadFile(file);
  auto scene = config_file.as<moveit_msgs::msg::PlanningScene>();
  for (auto &co : scene.world.collision_objects) {
    if (co.header.frame_id.empty()) {
      co.header.frame_id = "world";
    }
  }
  for (auto &fft : scene.fixed_frame_transforms) {
    if (fft.header.frame_id.empty()) {
      fft.header.frame_id = "map";
    }
  }
  return scene;
}

std::string load_xacro(std::string pkg, std::string path) {
  std::string pkg_path = ament_index_cpp::get_package_share_directory(pkg);
  std::string robot_description = pkg_path + path;
  std::cout << "Loading xacro file: " << robot_description << std::endl;
  std::shared_ptr<FILE> pipe(popen(("xacro " + robot_description).c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  char buffer[128];
  std::string result;
  while (!feof(pipe.get())) {
    if (fgets(buffer, 128, pipe.get()) != nullptr)
      result += buffer;
  }
  return result;
}

sensor_msgs::msg::JointState get_goal_state_from_request(const std::string &file) {
  YAML::Node config_file = YAML::LoadFile(file);
  auto joint_constraints = config_file["goal_constraints"][0]["joint_constraints"]
      .as<std::vector<moveit_msgs::msg::JointConstraint>>();
  sensor_msgs::msg::JointState goal_state;
  for (auto &jc : joint_constraints) {
    goal_state.name.push_back(jc.joint_name);
    goal_state.position.push_back(jc.position);
  }
  return goal_state;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string exp = "bookshelf_tall_ur5";
  //std::string exp = "box_ur5";
  std::string no = "0001";
  std::string path = ament_index_cpp::get_package_share_directory("benchmark");
  std::string file = path + "/motion_bench_maker/" + exp + "/scene" + no + ".yaml";
  auto scene = load_scene(file);
  std::string robot_description = load_xacro("robowflex_resources",
                                             "/ur/robots/ur5_robotiq_robot_limited_small_table.urdf.xacro");
  std_msgs::msg::String robot_description_msg;
  robot_description_msg.data = robot_description;
  std::string robot_description_semantic =
      load_xacro("robowflex_resources", "/ur/config/ur5/ur5_robotiq85_table.srdf.xacro");
  std_msgs::msg::String robot_description_semantic_msg;
  robot_description_semantic_msg.data = robot_description_semantic;

  // load request
  std::string request_file = path + "/motion_bench_maker/" + exp + "/request" + no + ".yaml";
  auto goal_state = get_goal_state_from_request(request_file);

  auto node = rclcpp::Node::make_shared("planning_scene_publisher");
  auto state_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  auto planning_scene_pub =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene",
                                                              rclcpp::QoS(1)
                                                                  .transient_local());
  auto robot_description_pub =
      node->create_publisher<std_msgs::msg::String>("robot_description",
                                                    rclcpp::QoS(1).transient_local());
  auto robot_description_semantic_pub =
      node->create_publisher<std_msgs::msg::String>("robot_description_semantic",
                                                    rclcpp::QoS(1).transient_local());

  robot_description_pub->publish(robot_description_msg);
  robot_description_semantic_pub->publish(robot_description_semantic_msg);
  planning_scene_pub->publish(scene);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    planning_scene_pub->publish(scene);
    goal_state.header.stamp = node->now();
    state_pub->publish(goal_state);
  }

  return 0;
}
