"""Launch the MuJoCo EVT2 simulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model', default_value='evt2',
        description='Robot model name registered in simulator_view_asyn.py')
    config_arg = DeclareLaunchArgument(
        'robot_config', default_value='full',
        description='Robot DOF configuration: full or 21')

    mujoco_node = Node(
        package='mujoco_node',
        executable='mujoco_node',
        name='mujoco_simulator_dex',
        output='screen',
        arguments=['-m', LaunchConfiguration('model'),
                   '-c', LaunchConfiguration('robot_config')],
    )

    return LaunchDescription([model_arg, config_arg, mujoco_node])
