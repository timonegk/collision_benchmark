from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch(context, *args, **kwargs):
    return [
        Node(package='benchmark',
             executable='motion_bench_maker_importer',
             name='benchmark',
             output='screen',
             )
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch)])
