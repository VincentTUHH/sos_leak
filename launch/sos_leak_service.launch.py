from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()

    launch_description.add_action(
        DeclareLaunchArgument('vehicle_name', default_value='klopsi00'))
    launch_description.add_action(DeclareLaunchArgument('tube_name'))

    launch_description.add_action(
        DeclareLaunchArgument('start_leak_server', default_value='false'))

    group = GroupAction([
        Node(package='sos_leak',
             executable='leak_server.py',
             namespace=LaunchConfiguration('vehicle_name'),
             condition=IfCondition(LaunchConfiguration('start_leak_server'))),
        Node(package='sos_leak',
             executable='leak_safety_service.py',
             namespace=LaunchConfiguration('vehicle_name'),
             condition=IfCondition(LaunchConfiguration('start_leak_server'))),
        Node(
            package='sos_leak',
            executable='leak_detector_service.py',
            namespace=LaunchConfiguration('vehicle_name'),
            parameters=[{
                'tube_name': LaunchConfiguration('tube_name'),
                'vehicle_name': LaunchConfiguration('vehicle_name'),
            }],
        ),
    ])

    launch_description.add_action(group)
    return launch_description
