#pragma once

#include <reach/interfaces/ik_solver.h>
#include <rclcpp/publisher.hpp>
#include <vector>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <functional>
#include <reach_ros/ik/moveit_ik_solver.h>
#include <boost/signals2.hpp>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
class JointModelGroup;
class RobotState;
}  // namespace core
}  // namespace moveit

namespace planning_scene
{
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene


namespace benchmark {
class MoveItIKSolver : public reach::IKSolver
{
 public:
  MoveItIKSolver(moveit::core::RobotModelConstPtr model, const std::string& planning_group, double dist_threshold);

  std::vector<std::vector<double>> solveIK(const Eigen::Isometry3d& target,
                                           const std::map<std::string, double>& seed) const override;

  std::vector<std::string> getJointNames() const override;

  void setTouchLinks(const std::vector<std::string>& touch_links);
  void addCollisionMesh(const std::string& collision_mesh_filename, const std::string& collision_mesh_frame);
  std::string getKinematicBaseFrame() const;

 protected:
  bool isIKSolutionValid(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                         const double* ik_solution) const;

  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  const double distance_threshold_;

  planning_scene::PlanningScenePtr scene_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  static std::string COLLISION_OBJECT_NAME;
};

struct MoveItIKSolverFactory : public reach::IKSolverFactory
{
  reach::IKSolver::ConstPtr create(const YAML::Node& config) const override;
};

}  // namespace reach_ros
