#!/usr/bin/env python3
"""MoveIt2 bringup for the JeTank arm.

Brings up everything MoveIt needs **except** the robot description
broadcaster: ros2_control_node, the joint_state_broadcaster +
arm_controller + gripper_controller spawners, and move_group. Optionally
launches RViz with the MoveIt motion-planning plugin.

This is the single source of truth for MoveIt orchestration. Other
launch files (``demo.launch.py``, ``jetank_ros_main/launch/unified.launch.py``)
include this file instead of re-declaring the configuration.

Assumes the caller has already started a ``robot_state_publisher`` and
broadcast any virtual joint TF (``world`` -> ``base_footprint``).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_moveit_configs(hardware='mock'):
    """Construct the MoveItConfigs object.

    Kept in a helper so we can reference the same configuration object
    from move_group and the optional RViz node. ``hardware`` selects the
    ros2_control backend baked into the robot_description (mock | serial);
    move_group, the controller manager and RViz must all see the same
    description, which is why this is resolved once here.
    """
    # Import here so consumers that never enable MoveIt do not need
    # ``moveit_configs_utils`` on PYTHONPATH at module-load time.
    from moveit_configs_utils import MoveItConfigsBuilder

    description_pkg = get_package_share_directory('jetank_description')
    robot_description_file = os.path.join(
        description_pkg, 'urdf', 'jetank_ros2_control.urdf.xacro'
    )

    return (
        MoveItConfigsBuilder('jetank', package_name='jetank_moveit_config')
        .robot_description(
            file_path=robot_description_file,
            mappings={'hardware': hardware},
        )
        .robot_description_semantic(file_path='config/jetank.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    # Resolve the hardware backend so the robot_description baked into
    # move_group / controller_manager / RViz is consistent.
    hardware = LaunchConfiguration('hardware').perform(context)

    moveit_config = _build_moveit_configs(hardware=hardware)

    # ros2_control controller manager. Mirrors the moveit_configs_utils
    # canonical pattern exactly: robot_description dict + the controllers
    # YAML as a PLAIN string path (so its top-level per-controller sections
    # stay top-level and reach the controllers). The node MUST be named
    # 'controller_manager' (spawners target it; the YAML namespaces its
    # controller-manager params under 'controller_manager:').
    controllers_file = os.path.join(
        get_package_share_directory('jetank_motor_control'),
        'config',
        'jetank_controllers.yaml',
    )
    # Canonical moveit_configs_utils pattern: robot_description (passed as a
    # parameter for determinism — the /robot_description topic has multiple
    # publishers here) plus the controllers YAML. The node MUST be named
    # 'controller_manager' (spawners target it; the YAML namespaces its
    # controller-manager params under 'controller_manager:').
    #
    # NOTE (controller_manager 2.54, RoboStack): joint_state_broadcaster
    # activates, but arm_controller / gripper_controller currently fail to
    # ingest their top-level `joints`/`command_interfaces` params at configure
    # time ("'joints' parameter is empty"), so trajectory execution is not yet
    # functional on this build. Planning in RViz works. See the plan's status
    # note for the open investigation.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            controllers_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'{controller}_spawner',
            arguments=[controller, '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}],
        )
        for controller in ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')
    ]

    # MoveIt move_group. Wrap the XML parameters so launch_ros does not try
    # to YAML-parse them.
    moveit_params = moveit_config.to_dict()
    for xml_key in ('robot_description', 'robot_description_semantic'):
        if xml_key in moveit_params and isinstance(moveit_params[xml_key], str):
            moveit_params[xml_key] = ParameterValue(moveit_params[xml_key], value_type=str)

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_params, {'use_sim_time': use_sim_time}],
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('moveit_ros_visualization'), 'launch', 'moveit.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return [
        ros2_control_node,
        *spawners,
        move_group_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz with the MoveIt motion-planning plugin',
        ),
        DeclareLaunchArgument(
            'hardware',
            default_value='mock',
            description='ros2_control backend baked into the robot_description: '
                        'mock (software-only, no motors) | serial (real servos)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
