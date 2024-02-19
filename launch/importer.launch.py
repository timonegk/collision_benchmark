from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch(context, *args, **kwargs):
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            # "request_adapters":
            # """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            # "start_state_max_bounds_error": 0.1,
        }
    }
    return [
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
        )
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch)])
