"""Launch the sim_joy Tkinter GUI teleop node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_joy',
            executable='teleop_gui',
            name='sim_joy_gui',
            output='screen',
        ),
    ])
