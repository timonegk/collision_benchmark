from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


parameters = [
  {'name': 'robot_description_file',          'description': 'Path to the URDF/xacro file',                     'default': PathJoinSubstitution([FindPackageShare('elise_description'), 'urdf', 'robot.urdf.xacro'])},
  {'name': 'robot_description_semantic_file', 'description': 'Path to the SRDF file',                           'default': PathJoinSubstitution([FindPackageShare('elise_moveit_config'), 'config', 'robot.srdf'])},
  {'name': 'use_rviz',                        'description': 'Flag indicating whether Rviz should be launchd',  'default': 'True'},
  {'name': 'rviz_config',                     'description': 'Reach study Rviz configuration',                  'default': PathJoinSubstitution([FindPackageShare('benchmark'), 'launch', 'reach_study_config.rviz'])},
]


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('robot_description_file')]), value_type=str)
    nodes = [
      # Robot state publisher
      Node(package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           output='screen',
           parameters=[{'robot_description': robot_description}]),
      # Joint state publisher
      Node(package='joint_state_publisher',
           executable='joint_state_publisher',
           name='joint_state_publisher',
           output='screen',
           parameters=[{'source_list': ParameterValue(['reach_joints'])}]),
      # Rviz
      Node(condition=IfCondition(LaunchConfiguration('use_rviz')),
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           arguments=['-d', LaunchConfiguration('rviz_config')],
           output='screen')
    ]

    return LaunchDescription(declare_launch_arguments() + nodes)
