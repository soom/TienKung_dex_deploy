from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_joy',
            executable='teleop_joy',
            name='sim_joy_gamepad',
            output='screen',
            parameters=[{'publish_rate': 30.0}],
            remappings=[],
        ),
    ])
