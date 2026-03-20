"""
compliance_control.launch.py
────────────────────────────
Generic ros2_control admittance compliance launch file.

Architecture (Humble topic-bridge):
  User/MoveIt ──(FollowJointTrajectory action)──► trajectory_bridge
                                                       │ JointTrajectoryPoint
                                                       ▼
                                              /admittance_controller/joint_references
                                                       │
                                              admittance_controller
                                                       │ position command
                                                       ▼
                                            hardware interface (mock or MuJoCo)

Launch arguments:
  use_mock_hardware    [false]  — Use mock_components/GenericSystem (true) or
                                  mujoco_ros2_control/MuJoCoSystem (false).
  xacro_file           — Path to robot xacro file.
                         Default: piper_description/urdf/piper_description_mujoco.xacro
  controllers_config   — Path to ros2_controllers yaml.
                         Default: compliance_control/config/ros2_controllers.yaml
  viewer_package       — Package containing the MuJoCo viewer executable.
                         Default: piper_mujoco
  viewer_executable    — MuJoCo viewer executable name.
                         Default: piper_mujoco_ctrl.py

Controller load order:
  1. joint_state_broadcaster
  2. force_torque_sensor_broadcaster
  3. admittance_controller + gripper_controller (+ FT injectors when mock)

Debugging:
  Set DEBUG_NODE env var to the executable name of the node to debug.
  The node will pause and wait for a debugpy attach on port 5678 (VS Code F5).

  Examples:
    DEBUG_NODE=piper_mujoco_ctrl.py             ros2 launch compliance_control compliance_control.launch.py
    DEBUG_NODE=admittance_trajectory_bridge.py  ros2 launch compliance_control compliance_control.launch.py
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node


def remove_comments(text):
    """Strip XML comments (some parsers choke on them in robot_description)."""
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def debug_prefix(executable_name):
    """Return debugpy prefix if DEBUG_NODE matches this executable, else empty string."""
    if os.environ.get('DEBUG_NODE') == executable_name:
        return 'python3 -m debugpy --listen 5678 --wait-for-client'
    return ''


def generate_launch_description():
    pkg_description = get_package_share_directory('piper_description')
    pkg_compliance  = get_package_share_directory('compliance_control')

    # ── Default paths ─────────────────────────────────────────────────────────
    default_xacro_file = os.path.join(
        pkg_description, 'urdf', 'piper_description_mujoco.xacro')
    default_controllers_config = os.path.join(
        pkg_compliance, 'config', 'ros2_controllers.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description=(
            'Use mock_components/GenericSystem (true) or '
            'mujoco_ros2_control/MuJoCoSystem (false).'
        ),
    )
    xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        default_value=default_xacro_file,
        description='Path to the robot xacro file.',
    )
    controllers_config_arg = DeclareLaunchArgument(
        'controllers_config',
        default_value=default_controllers_config,
        description='Path to the ros2_controllers yaml configuration file.',
    )
    viewer_package_arg = DeclareLaunchArgument(
        'viewer_package',
        default_value='piper_mujoco',
        description='Package that contains the MuJoCo viewer executable.',
    )
    viewer_executable_arg = DeclareLaunchArgument(
        'viewer_executable',
        default_value='piper_mujoco_ctrl.py',
        description='Name of the MuJoCo viewer executable.',
    )

    use_mock_hardware   = LaunchConfiguration('use_mock_hardware')
    xacro_file          = LaunchConfiguration('xacro_file')
    controllers_config  = LaunchConfiguration('controllers_config')
    viewer_package      = LaunchConfiguration('viewer_package')
    viewer_executable   = LaunchConfiguration('viewer_executable')

    # ── Robot description (resolved at launch time using default xacro path) ─
    # NOTE: xacro processing happens eagerly here using the default path.
    # For non-default xacro files pass xacro_file:=<path> and ensure the
    # mappings below match your robot's xacro parameters.
    doc = xacro.parse(open(default_xacro_file))
    xacro.process_doc(doc, mappings={'use_mock_hardware': 'true',
                                      'lock_joints_4_6': 'true'})
    robot_description_default = remove_comments(doc.toxml())

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_default},
            {'publish_frequency': 50.0},
        ],
    )

    # ── ros2_control_node ─────────────────────────────────────────────────────
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_default},
            default_controllers_config,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # ── MuJoCo visualization node (both modes) ────────────────────────────────
    # viewer_only=true  when use_mock_hardware=false (mujoco_ros2_control runs physics)
    # viewer_only=false when use_mock_hardware=true  (viewer drives physics itself)
    mujoco_viewer = Node(
        package=viewer_package,
        executable=viewer_executable,
        output='screen',
        prefix=debug_prefix('piper_mujoco_ctrl.py'),
        parameters=[{'viewer_only': NotSubstitution(use_mock_hardware)}],
    )

    # ── Trajectory bridge (FollowJointTrajectory action → admittance topic) ──
    trajectory_bridge = Node(
        package='compliance_control',
        executable='admittance_trajectory_bridge.py',
        output='screen',
        prefix=debug_prefix('admittance_trajectory_bridge.py'),
    )

    # ── World-frame force bridge (world Wrench → sensor-frame injectors) ──────
    world_force_bridge = Node(
        package='compliance_control',
        executable='world_force_bridge.py',
        output='screen',
        prefix=debug_prefix('world_force_bridge.py'),
        condition=IfCondition(use_mock_hardware),
    )

    # ── Controller spawners (ordered) ─────────────────────────────────────────
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    spawn_ft_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['force_torque_sensor_broadcaster'],
        output='screen',
    )

    spawn_admittance = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['admittance_controller'],
        output='screen',
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    # FT force injectors (mock hardware only — writes fake forces to sensor)
    spawn_ft_fx = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_fx_injector'],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )
    spawn_ft_fy = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_fy_injector'],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )
    spawn_ft_fz = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_fz_injector'],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )
    spawn_ft_tx = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_tx_injector'],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )
    spawn_ft_ty = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_ty_injector'],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )
    spawn_ft_tz = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ft_tz_injector'],
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )

    # ── Event chain: jsb → ft_broadcaster → admittance + gripper + injectors ─
    evt_ft = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ft_broadcaster],
        )
    )
    evt_admittance_and_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ft_broadcaster,
            on_exit=[spawn_admittance, spawn_gripper,
                     spawn_ft_fx, spawn_ft_fy, spawn_ft_fz,
                     spawn_ft_tx, spawn_ft_ty, spawn_ft_tz],
        )
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        xacro_file_arg,
        controllers_config_arg,
        viewer_package_arg,
        viewer_executable_arg,
        robot_state_publisher,
        ros2_control_node,
        mujoco_viewer,
        trajectory_bridge,
        world_force_bridge,
        spawn_jsb,
        evt_ft,
        evt_admittance_and_gripper,
    ])
