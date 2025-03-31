from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument('vehicle_name', default_value='klopsi00'))
    launch_description.add_action(DeclareLaunchArgument('tube_name'))

    group = GroupAction([
        # PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(
            package='sos_leak',
            executable='leak_detector_service',
            namespace=LaunchConfiguration('vehicle_name') + '/' +
            LaunchConfiguration('tube_name'),
            parameters=[{
                'tube_name': LaunchConfiguration('tube_name'),
            }],
        ),
        Node(
            package='sos_leak',
            executable='leak_safety_service',
            namespace=LaunchConfiguration('vehicle_name') + '/' +
            LaunchConfiguration('tube_name'),
            parameters=[{
                'tube_name': LaunchConfiguration('tube_name'),
            }],
        ),
        Node(package='sos_leak',
             executable='leak_server.py',
             namespace=LaunchConfiguration('vehicle_name')),
    ])

    launch_description.add_action(group)
    return launch_description
