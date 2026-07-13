#!/usr/bin/env python3
"""
MoveIt2 bringup for the JeTank arm.

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
    """
    Construct the MoveItConfigs object.

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
    # canonical pattern exactly: robot_description dict (passed as a parameter
    # for determinism — the /robot_description topic has multiple publishers
    # here) + the controllers YAML as a PLAIN string path (so its top-level
    # per-controller sections stay top-level and reach the controllers). The
    # node MUST be named 'controller_manager' (spawners target it; the YAML
    # namespaces its controller-manager params under 'controller_manager:').
    #
    # NOTE (controller_manager 2.54, RoboStack): joint_state_broadcaster
    # activates, but arm_controller / gripper_controller currently fail to
    # ingest their top-level `joints`/`command_interfaces` params at configure
    # time ("'joints' parameter is empty"), so trajectory execution is not yet
    # functional on this build. Planning in RViz works. See the plan's status
    # note for the open investigation.
    controllers_file = os.path.join(
        get_package_share_directory('jetank_motor_control'),
        'config',
        'jetank_controllers.yaml',
    )
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

    # controller_manager 2.54 loads each controller's params_file into the
    # controller NODE, where rclcpp only matches a bare `/**` wildcard key (the
    # controller name / `/<name>` / `/**/<name>` all fail on this build). So each
    # controller gets its OWN param file whose params live under `/**`; a single
    # shared file would leak every controller's params to every controller.
    # joint_state_broadcaster needs no params (it auto-discovers joints).
    motor_config = os.path.join(
        get_package_share_directory('jetank_motor_control'), 'config', 'controllers')
    controller_param_files = {
        'arm_controller': os.path.join(motor_config, 'arm_controller.yaml'),
        'gripper_controller': os.path.join(motor_config, 'gripper_controller.yaml'),
    }
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'{controller}_spawner',
            arguments=[controller, '--controller-manager', '/controller_manager'] +
                      (['--param-file', controller_param_files[controller]]
                       if controller in controller_param_files else []),
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

    # WORKAROUND (2026-06-28, robostack-staging mutex-0.9.0 skew): the standalone
    # ros2_control_node here runs core ros2_control 2.54.0 but the controllers are
    # 2.53.1. Across that boundary the controllers fail to register their own node
    # names and collapse onto the `controller_manager` node, so their action
    # servers come up at /controller_manager/<action> instead of
    # /<controller_name>/<action>. MoveIt's SimpleControllerManager builds the
    # client name as <controller_name>/<action_ns> (e.g.
    # /arm_controller/follow_joint_trajectory) and finds 0 servers -> every
    # execute aborts instantly with "Action client not connected".
    # Remap move_group's two action clients onto the namespace where the servers
    # actually live. NOTE: an action is 3 services + 2 topics under its base name,
    # and rcl only remaps those concrete sub-entities (a base-name remap like
    # `/arm_controller/follow_joint_trajectory:=...` matches nothing). So each of
    # the 5 sub-interfaces is remapped explicitly, per action.
    # Safe here because this file is ONLY the standalone CM path (mock/serial);
    # the sim path uses moveit_sim.launch.py with gz_ros2_control, which
    # namespaces controllers correctly and is untouched. Remove this whole block
    # once a coherent ros2_control snapshot is available (see pixi.toml note).
    def _action_remaps(client_action, server_action):
        return [
            (f'{client_action}/_action/{sub}', f'{server_action}/_action/{sub}')
            for sub in ('send_goal', 'cancel_goal', 'get_result', 'feedback', 'status')
        ]
    moveit_execution_remaps = (
        _action_remaps('/arm_controller/follow_joint_trajectory',
                       '/controller_manager/follow_joint_trajectory') +
        _action_remaps('/gripper_controller/gripper_cmd',
                       '/controller_manager/gripper_cmd')
    )
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_params, {'use_sim_time': use_sim_time}],
        remappings=moveit_execution_remaps,
    )

    rviz_config_file = LaunchConfiguration('rviz_config')
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
