#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <yaml-cpp/yaml.h>
#include "yaml.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // get path to "benchmark" from ament
  std::string path = ament_index_cpp::get_package_share_directory("benchmark");
  std::string file = path + "/motion_bench_maker/box_ur5/scene0001.yaml";
  YAML::Node node = YAML::LoadFile(file);
  moveit_msgs::msg::PlanningScene scene = node.as<moveit_msgs::msg::PlanningScene>();
  std::cout << scene.robot_model_name << std::endl;

  return 0;
}
