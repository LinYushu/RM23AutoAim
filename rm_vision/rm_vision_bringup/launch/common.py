import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

package_name = 'infantry_description'
urdf_name = "infantry_description.urdf"
pkg_share = FindPackageShare(package=package_name).find(package_name) 
urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))


robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    arguments=[urdf_model_path]
)

node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

tracker_node = Node(
    package='armor_tracker',
    executable='armor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level', 'armor_tracker:='+launch_params['tracker_log_level']],
)
