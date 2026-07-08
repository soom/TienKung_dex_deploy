"""Launch the rl_control node for real hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file', default_value='dex_config_real.yaml',
        description='Filename under share/rl_control/config to load')

    config_path = PathJoinSubstitution([
        FindPackageShare('rl_control'), 'config', LaunchConfiguration('config_file')
    ])

    rl_control = Node(
        package='rl_control',
        executable='rl_control_node',
        name='xmigcs_control_node',
        output='screen',
        parameters=[{'config_file': config_path}],
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('joystick'), '/launch/joystick.launch.py'
        ])
    )

    body_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('body_control'), '/launch/body_control.launch.py'
        ])
    )

    delayed_rl_control = TimerAction(
        period=3.0,
        actions=[rl_control],
    )

    return LaunchDescription([config_arg, body_control, joystick, delayed_rl_control])
