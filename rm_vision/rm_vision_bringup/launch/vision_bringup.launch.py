import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():

    from rm_vision_bringup.common import node_params, launch_params, robot_state_publisher, tracker_node
    
    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            composable_node_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(hik_camera_node, mv_camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                hik_camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    composable_node_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
            emulate_tty=True,
            remappings=[('armor_detector/armor_pos', '/armor_detector/armor_pos')],
            ros_args=['--log-level', launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_pkg = get_package_share_directory('hik_camera')
    mv_camera_pkg = get_package_share_directory('mindvision_camera')

    hik_camera_node = get_camera_node(hik_camera_pkg, 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node(mv_camera_pkg, 'mindvision_camera::MVCameraNode')

    if launch_params['camera'] == 'hik':
        cam_detector = get_camera_detector_container(hik_camera_node, mv_camera_node)
    elif launch_params['camera'] == 'mv':
        cam_detector = get_camera_detector_container(mv_camera_node, hik_camera_node)

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='screen',
        emulate_tty=True,
        parameters=[node_params],
        ros_args=['--log-level', launch_params['serial_log_level']],
        on_exit=Shutdown(),
    )

    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        delay_serial_node,
        delay_tracker_node,
    ])