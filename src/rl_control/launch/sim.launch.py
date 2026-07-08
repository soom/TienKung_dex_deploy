"""Launch the simulation stack: MuJoCo + sim_joy GUI + rl_control_node_sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file', default_value='dex_config_sim.yaml',
        description='Filename under share/rl_control/config to load')
    model_arg = DeclareLaunchArgument(
        'model', default_value='evt2',
        description='MuJoCo model registered name')
    robot_config_arg = DeclareLaunchArgument(
        'robot_config', default_value='full',
        description='Robot DOF configuration (full or 21)')

    config_path = PathJoinSubstitution([
        FindPackageShare('rl_control'), 'config', LaunchConfiguration('config_file')
    ])

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mujoco_node'), '/launch/mujoco_sim.launch.py'
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'robot_config': LaunchConfiguration('robot_config'),
        }.items(),
    )

    sim_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('sim_joy'), '/launch/teleop_gui.launch.py'
        ])
    )

    rl_control = Node(
        package='rl_control',
        executable='rl_control_node_sim',
        name='xmigcs_control_node',
        output='screen',
        parameters=[{'config_file': config_path}],
    )

    return LaunchDescription([
        config_arg,
        model_arg,
        robot_config_arg,
        mujoco_launch,
        sim_joy_launch,
        rl_control,
    ])
