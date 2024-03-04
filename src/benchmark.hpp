#pragma once

#include <climits>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <random_numbers/random_numbers.h>

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "benchmark_logger.hpp"
#include "benchmark_visualizer.hpp"

/**
 * @brief Class for Inverse Kinematics Benchmarking
 *
 * This class is responsible for benchmarking the performance
 * of Inverse Kinematics (IK) solvers. It runs a set of trials
 * to evaluate success rate, solve time, and error metrics.
 */
class IKBenchmarking {
  enum class IKInitializationType {
    RANDOM,   ///< Random joint values
    PREVIOUS, ///< Previous joint values
    DEFAULT   ///< Default joint values
  };
 public:
  /**
   * @brief Constructor using a shared pointer to an rclcpp::Node.
   *
   * @param node Shared pointer to the node.
   */
  explicit IKBenchmarking(rclcpp::Node::SharedPtr node)
      : node_(node),
        logger_(node->get_logger()),
        robot_model_loader_(node),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false) {
  }

  /**
   * @brief Overloaded constructor that creates a node.
   *
   * @param node_name The name of the node to create.
   * @param options Additional node options.
   */
  IKBenchmarking(const std::string& node_name,
                 const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node_(rclcpp::Node::make_shared(node_name, options)),
        logger_(node_->get_logger()),
        robot_model_loader_(node_),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false) {
  }

  /**
   * @brief Overloaded constructor to create a node, specify IK solver, and output file.
   *
   * @param node_name The name of the node to create.
   * @param solver The name of the IK solver to use.
   * @param output_file The name of the output file for data collection.
   * @param options Additional node options.
   */
  IKBenchmarking(const std::string& node_name, const std::string& solver,
                 const std::string output_file,
                 const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : node_(rclcpp::Node::make_shared(node_name, options)),
        logger_(node_->get_logger()),
        robot_model_loader_(node_),
        robot_model_(robot_model_loader_.getModel()),
        robot_state_(new moveit::core::RobotState(robot_model_)),
        calculation_done_(false) {
  }

  /**
   * @brief Start collecting benchmarking data.
   *
   * This function calls other member functions responsible for
   * initializing relevant member variables start the process of
   * collecting IK solving data.
   */
  void run(const std::string &log_file);

  /**
   * @brief Get the success rate of the solver.
   *
   * @return Success rate as a value between 0 and 1, representing
   * the ratio of successful trials to the total IK solution trials.
   */
  double get_success_rate() const;

  /**
   * @brief Get the average solve time.
   *
   * @return Average solve time in microseconds.
   */
  double get_average_solve_time() const;

  /**
   * @brief Check if the benchmarking calculation is done.
   *
   * @return True if done, false otherwise.
   */
  bool calculation_done() const;

 private:
  rclcpp::Node::SharedPtr node_;  ///< Shared pointer to a ROS 2 node.
  rclcpp::Logger logger_;         ///< ROS 2 logger for this class.

  robot_model_loader::RobotModelLoader
      robot_model_loader_;  ///< Loader for the robot model to be utilized for solving IK.
  moveit::core::RobotModelPtr robot_model_;  ///< Shared pointer to the robot model to be loaded
  ///< using the robot_model_loader_.
  moveit::core::RobotStatePtr robot_state_;  ///< Shared pointer to the robot state which carries
  ///< information about joint values.

  std::string planning_group_name_;  ///< The name of the planning group (move_group) for which to
  ///< compute IK.
  const moveit::core::JointModelGroup*
      joint_model_group_;  ///< Pointer to the joint model group used to get information about the
  ///< group and solve IK.
  std::string
      tip_link_name_;  ///< The name of the tip link in the planning group used to solve IK.

  random_numbers::RandomNumberGenerator
      generator_;  ///< Generator for random joint values within bounds.

  size_t sample_size_;  ///< The number of samples to run for collecting benchmarking data.
  double ik_timeout_;  ///< Maximum time (in seconds) allowed for an IK solver to find a solution.
  size_t ik_iteration_display_step_;  ///< Number of IK solve attempts between displaying updates.
  bool calculation_done_;         ///< Flag to indicate whether the calculation is done.

  std::shared_ptr<BenchmarkLogger> benchmarking_logger_;  ///< Shared pointer to the benchmark logger.
  std::shared_ptr<BenchmarkVisualizer> benchmarking_visualizer_;  ///< Shared pointer to the benchmark visualizer.

  /**
   * @brief Initialize the relevant class variables and prepare for benchmarking.
   */
  void initialize(const std::string& log_file);

  /**
   * @brief Collect and save the benchmarking data.
   */
  void gather_data();
};
