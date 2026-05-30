#!/usr/bin/env python3
"""Stand-alone MoveIt2 demo for the JeTank arm.

Brings up everything needed to plan motions in RViz on a developer
machine: robot_state_publisher + virtual joint TF + the full
``moveit_bringup`` (controller manager, spawners, move_group, RViz).

For the integrated robot launch use
``ros2 launch jetank_ros_main unified.launch.py enable_moveit:=true``;
that launch file includes the same ``moveit_bringup`` plus the rest of
the JeTank stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    hardware = LaunchConfiguration('hardware')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the MoveIt motion-planning plugin',
    )
    declare_hardware = DeclareLaunchArgument(
        'hardware',
        default_value='mock',
        description='ros2_control backend: mock (software-only, no motors) | '
                    'serial (real servos)',
    )

    pkg_jetank_moveit = get_package_share_directory('jetank_moveit_config')
    pkg_jetank_description = get_package_share_directory('jetank_description')

    # robot_state_publisher with the same URDF MoveIt sees.
    robot_description_file = os.path.join(
        pkg_jetank_description, 'urdf', 'jetank_ros2_control.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', robot_description_file, ' hardware:=', hardware]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Static TF: world -> base_footprint (virtual joint of the arm group).
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='virtual_joint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_jetank_moveit, 'launch', 'moveit_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'hardware': hardware,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_hardware,
        robot_state_publisher,
        static_tf,
        moveit_bringup,
    ])
