#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // get path to "benchmark" from ament
  std::string path = ament_index_cpp::get_package_share_directory("benchmark");
  std::string file = path + "/motion_bench_maker/box_ur5/scene0001.yaml";
  auto scene = load_scene(file);
  std::string robot_description = load_xacro("robowflex_resources",
                                             "/ur/robots/ur5_robotiq_robot_limited_small_table.urdf.xacro");
  std_msgs::msg::String robot_description_msg;
  robot_description_msg.data = robot_description;
  std::string robot_description_semantic =
      load_xacro("robowflex_resources", "/ur/config/ur5/ur5_robotiq85_table.srdf.xacro");
  std_msgs::msg::String robot_description_semantic_msg;
  robot_description_semantic_msg.data = robot_description_semantic;

  auto node = rclcpp::Node::make_shared("planning_scene_publisher");
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
  }

  return 0;
}
