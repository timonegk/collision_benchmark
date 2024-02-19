from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch(context, *args, **kwargs):
    moveit_config = MoveItConfigsBuilder("elise").robot_description_kinematics(file_path=PathJoinSubstitution([FindPackageShare('benchmark'), 'config', 'kinematics.yaml']).perform(context)).to_moveit_configs()


    return [
            # Robot state publisher
            Node(package='robot_state_publisher',
                 executable='robot_state_publisher',
                 name='robot_state_publisher',
                 output='screen',
                 parameters=[moveit_config.robot_description]),
            Node(package='benchmark',
                 executable='benchmark_node',
                 name='benchmark',
                 output='screen',
                 parameters=[
                     moveit_config.robot_description,
                     moveit_config.robot_description_semantic,
                     moveit_config.robot_description_kinematics,
                     {'random_seed': 0,
                      'planning_group': 'arm',
                      'sample_size': 1000,
                      'ik_timeout': 0.1,
                      'ik_iteration_display_step': 1,
                      }]),
        ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch)])
