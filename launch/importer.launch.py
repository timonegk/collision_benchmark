from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def launch(context, *args, **kwargs):
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("robowflex_resources"),
                    "ur", "robots", "ur5_robotiq_robot_limited_small_table.urdf.xacro",
                ),
            ],
            on_stderr="ignore",
        ),
        value_type=str,
    )


    return [
        Node(
            package="benchmark",
            executable="motion_bench_maker_importer",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        #Node(
        #    package="joint_state_publisher_gui",
        #    executable="joint_state_publisher_gui",
        #    output="screen",
        #),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", os.path.join(get_package_share_directory(
                "benchmark"), "config", "planning.rviz")],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                {"default_planning_pipeline": "ompl",
                 "planning_pipelines": ["ompl"],
                 "ompl": {
                     "planning_plugins": ["ompl_interface/OMPLPlanner"],
                 }
                 }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch)])
