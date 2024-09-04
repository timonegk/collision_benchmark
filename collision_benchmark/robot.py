from ament_index_python.packages import get_package_share_directory
import reach_ros
from ebike.utils import get_xacro
from ebike.robot import AbstractRobot


class Elise(AbstractRobot):
    name = 'ELISE'

    def set_config(self):
        xacro_file = get_package_share_directory('elise_description') + '/urdf/robot.urdf.xacro'
        robot_description = get_xacro(xacro_file)
        semantic_file = get_package_share_directory('elise_moveit_config') + '/config/robot.srdf'
        with open(semantic_file, 'r') as file:
            robot_description_semantic = file.read()
        reach_ros.set_parameter('robot_description', robot_description)
        reach_ros.set_parameter('robot_description_semantic', robot_description_semantic)

    def get_planning_group(self):
        return "all"
