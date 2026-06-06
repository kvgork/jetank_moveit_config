#!/usr/bin/env python3
"""MoveIt2 motion planning + execution for the JeTank arm IN GAZEBO.

Unlike ``moveit_bringup.launch.py`` (which starts its own standalone
``ros2_control_node`` for mock/serial hardware), this launch runs MoveIt on
top of the controller manager **provided by Gazebo** (``ign_ros2_control``).
It therefore launches:

  1. the Gazebo simulation (``jetank_simulation``) with the arm controller
     started active — this brings up the controller manager,
     joint_state_broadcaster, diff_drive, gripper and arm_controller, and the
     robot_state_publisher;
  2. ``move_group`` only — NO second ros2_control_node, NO spawners.

move_group plans for the ``arm`` group and executes via the
``/arm_controller/follow_joint_trajectory`` action that the Gazebo-side
arm_controller exposes (see config/moveit_controllers.yaml).

    ros2 launch jetank_moveit_config moveit_sim.launch.py            # headless
    ros2 launch jetank_moveit_config moveit_sim.launch.py use_rviz:=true headless:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_moveit_configs():
    """MoveItConfigs with the SAME description Gazebo uses (use_sim:=true)."""
    from moveit_configs_utils import MoveItConfigsBuilder

    description_pkg = get_package_share_directory('jetank_description')
    robot_description_file = os.path.join(
        description_pkg, 'urdf', 'jetank_ros2_control.urdf.xacro'
    )
    return (
        MoveItConfigsBuilder('jetank', package_name='jetank_moveit_config')
        .robot_description(
            file_path=robot_description_file,
            mappings={'use_sim': 'true', 'use_ros2_control': 'true'},
        )
        .robot_description_semantic(file_path='config/jetank.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            # Gazebo's robot_state_publisher already publishes /robot_description,
            # so move_group should not publish a second (latched) copy.
            publish_robot_description=False,
            publish_robot_description_semantic=True,
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )


def launch_setup(context, *args, **kwargs):
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    start_gazebo = LaunchConfiguration('start_gazebo').perform(context).lower() in ('true', '1')

    moveit_config = _build_moveit_configs()

    sim_launch = 'gazebo_headless.launch.py' if headless else 'gazebo.launch.py'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('jetank_simulation'), 'launch', sim_launch,
        ])),
        launch_arguments={
            'use_sim_time': 'true',
            'start_arm_active': 'true',
        }.items(),
    )

    # move_group params. Wrap the XML descriptions so launch_ros does not try
    # to YAML-parse them.
    params = moveit_config.to_dict()
    for xml_key in ('robot_description', 'robot_description_semantic'):
        if isinstance(params.get(xml_key), str):
            params[xml_key] = ParameterValue(params[xml_key], value_type=str)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[params, {'use_sim_time': True}],
    )

    rviz_config_file = LaunchConfiguration('rviz_config')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(use_rviz),
    )

    # When start_gazebo is false, attach move_group to an already-running
    # Gazebo (e.g. launched by jetank_ros_main/sim_demo.launch.py).
    actions = []
    if start_gazebo:
        actions.append(gazebo)
    actions += [move_group_node, rviz_node]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz with the MoveIt motion-planning plugin',
        ),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run Gazebo headless (no GUI)',
        ),
        DeclareLaunchArgument(
            'start_gazebo', default_value='true',
            description='Launch Gazebo here. Set false to run move_group only '
                        'against an already-running simulation.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('jetank_moveit_config'), 'config', 'moveit.rviz']),
            description='RViz config to load. Defaults to the jetank config with '
                        'RobotModel + MotionPlanning panel (the upstream '
                        'moveit_ros_visualization moveit.rviz is bare — no displays).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
