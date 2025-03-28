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
        DeclareLaunchArgument(
            'vehicle_name', 
            default_value='klopsi00'
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            'tube_name'
        )
    )

    group = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration('vehicle_name')),
            Node(
                executable='leak_detector.py',
                package='sos_leak',
                parameters=[
                    {
                        'tube_name': LaunchConfiguration('tube_name')
                    },
                ],
            ),
            
            Node(
                executable='leak_safety.py',
                package='sos_leak',
            ),
        ]
    )

    launch_description.add_action(group)
    return launch_description
