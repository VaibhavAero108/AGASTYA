# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Vaibhav (VaibhavAero108)
"""One-shot AGASTYA launch: Gazebo world + vehicle, ROS bridge, RViz (FPV), control pad.

    ros2 launch agastya_sim simulation.launch.py            # everything
    ros2 launch agastya_sim simulation.launch.py gui:=false # headless server (tests/CI)
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def select_gl(context):
    """Pick the OpenGL driver for Gazebo / RViz under WSLg.

    WSLg's Mesa otherwise falls back to llvmpipe (CPU rendering). 'nvidia' /
    'intel' use Mesa's D3D12 driver on that Windows GPU, 'software' forces
    llvmpipe, 'system' leaves the environment untouched.
    """
    choice = LaunchConfiguration('gl').perform(context).strip().lower()
    if choice == 'system' or not os.path.exists('/dev/dxg') and choice != 'software':
        return []
    if choice == 'software':
        return [SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
                SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe')]
    adapter = {'nvidia': 'NVIDIA', 'intel': 'Intel', 'amd': 'AMD'}.get(choice, choice)
    return [SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
            SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', adapter)]


def generate_launch_description():
    description_share = get_package_share_directory('agastya_description')
    sim_share = get_package_share_directory('agastya_sim')
    plugin_lib = os.path.join(get_package_prefix('agastya_gz_plugins'), 'lib')

    world = os.path.join(description_share, 'worlds', 'agastya_dual_domain.sdf')
    urdf = os.path.join(description_share, 'urdf', 'agastya.urdf.xacro')
    bridge_config = os.path.join(sim_share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(sim_share, 'config', 'agastya.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    pad = LaunchConfiguration('pad')
    use_sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' link_mode:=', LaunchConfiguration('link')]), value_type=str)

    gz_args = PythonExpression([
        "'-r -v 2 ", world, "' if '", gui, "' == 'true' else '-r -s -v 2 --headless-rendering ", world, "'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}, use_sim_time],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'agastya_dual_domain', '-name', 'agastya', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_config}, use_sim_time],
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config], output='log',
        parameters=[use_sim_time], condition=IfCondition(PythonExpression(["'", gui, "' == 'true' and '", rviz, "' == 'true'"])),
    )
    markers = Node(package='agastya_sim', executable='world_markers', output='log', parameters=[use_sim_time])
    link_sim = Node(package='agastya_sim', executable='link_sim', output='log', parameters=[use_sim_time])
    control_pad = Node(
        package='agastya_sim', executable='control_pad', output='screen',
        condition=IfCondition(PythonExpression(["'", gui, "' == 'true' and '", pad, "' == 'true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI + RViz + control pad'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz with robot model and FPV'),
        DeclareLaunchArgument('pad', default_value='true', description='Open the keyboard control pad window'),
        DeclareLaunchArgument('x', default_value='5.5', description='Spawn x (landing pad)'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.25'),
        DeclareLaunchArgument('yaw', default_value='3.14159', description='Spawn heading (facing the pool)'),
        DeclareLaunchArgument('link', default_value='fibre',
                              description='Command link: fibre (tether, works underwater) | radio (lost underwater)'),
        DeclareLaunchArgument('gl', default_value='nvidia',
                              description='OpenGL under WSLg: nvidia | intel | software | system'),
        OpaqueFunction(function=select_gl),
        # Gazebo must find package:// meshes and the AgastyaDynamics system.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description_share)),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_lib),
        gazebo,
        state_publisher,
        bridge,
        markers,
        link_sim,
        TimerAction(period=4.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[rviz_node, control_pad]),
    ])
