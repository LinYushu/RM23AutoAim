
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Load and include the first launch file
    # package1_path = os.path.join(get_package_share_directory(
    #     'mindvision_camera'))
    # launch_file1 = os.path.join(package1_path, 'launch', 'mv_launch.py')
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_file1)
    # ))

    # # Load and include the second launch file
    # package2_path = os.path.join(get_package_share_directory(
    #     'rm_serial_driver'))
    # launch_file2 = os.path.join(package2_path, 'launch', 'serial_driver.launch.py')
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_file2)
    # ))

    # # Load and include the third launch file
    # package3_path = os.path.join(get_package_share_directory('auto_aim_bringup'))
    # launch_file3 = os.path.join(package3_path, 'launch', 'auto_aim.launch.py')
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_file3)
    # ))

    # # Load and include the fourth launch file
    # package4_path = os.path.join(get_package_share_directory('infantry_description'))
    # launch_file4 = os.path.join(package4_path, 'launch', 'view_model.launch.py')
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(launch_file4)
    # ))


#    package5_path = os.path.join(get_package_share_directory('record_node'))
#    launch_file5 = os.path.join(package5_path,  'record.launch.py')
#    ld.add_action(IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(launch_file5)
#    ))
    foxglove_bridge_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('foxglove_bridge'), 'ros2_foxglove_bridge','launch', 'foxglove_bridge_launch.xml')
        )
    )
    ld.add_action(foxglove_bridge_launch)
   

    return ld
