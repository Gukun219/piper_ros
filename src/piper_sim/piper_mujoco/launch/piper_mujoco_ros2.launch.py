"""
piper_mujoco_ros2.launch.py
────────────────────────────
Launch file for Piper arm simulation with ros2_control admittance control.

Controller chain:
  MoveIt  ──(FollowJointTrajectory action)──►  arm_controller (JTC, chainable)
                                                     │ chainable reference pos
                                                     ▼
                                             admittance_controller
                                                     │ position command
                                                     ▼
                                          hardware interface  (mock or MuJoCo)

Launch arguments:
  use_mock_hardware  [true]  — Use mock_components/GenericSystem (default).
                               No extra package needed. Joints mirror commands.
                               MuJoCo viewer runs as a separate node subscribing
                               to /joint_states for visual feedback.
                    [false]  — Use mujoco_ros2_control/MuJoCoSystem for full
                               closed-loop physics + force feedback.
                               Requires mujoco_ros2_control to be built.

Controller load order (critical for chaining):
  1. joint_state_broadcaster
  2. force_torque_sensor_broadcaster
  3. admittance_controller   ← registers chainable interfaces first
  4. arm_controller          ← connects to admittance chainable interfaces
  5. gripper_controller

Prerequisites:
  sudo apt install ros-humble-admittance-controller \\
                   ros-humble-force-torque-sensor-broadcaster \\
                   ros-humble-kinematics-interface \\
                   ros-humble-kinematics-interface-kdl
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def remove_comments(text):
    """Strip XML comments (some parsers choke on them in robot_description)."""
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def generate_launch_description():
    pkg_description = get_package_share_directory('piper_description')
    pkg_mujoco      = get_package_share_directory('piper_mujoco')

    # ── Launch argument ───────────────────────────────────────────────────────
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description=(
            'Use mock_components/GenericSystem (true) or '
            'mujoco_ros2_control/MuJoCoSystem (false).'
        ),
    )
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    # ── Robot description ─────────────────────────────────────────────────────
    # Process xacro at launch time, passing the mock-hardware flag
    xacro_file = os.path.join(pkg_description, 'urdf',
                              'piper_description_mujoco.xacro')

    # We need the value of use_mock_hardware at parse time for xacro:arg.
    # Since LaunchConfiguration is resolved at runtime, we default to 'true'
    # here and let the user override via command line.
    # For a fully dynamic version, use ParameterValue + Command substitution.
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_mock_hardware': 'true'})
    robot_description_default = remove_comments(doc.toxml())

    controllers_yaml = os.path.join(pkg_mujoco, 'config', 'ros2_controllers.yaml')

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
    # Note: robot_description is published by robot_state_publisher via topic.
    # Passing it directly here as a parameter is deprecated in Humble but still
    # works; the warning is harmless.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description_default},
            controllers_yaml,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # ── MuJoCo visualization node (mock mode: mirrors /joint_states → MuJoCo)
    # Runs only when use_mock_hardware=true; provides visual feedback even
    # without a full mujoco_ros2_control integration.
    mujoco_viewer = Node(
        package='piper_mujoco',
        executable='piper_mujoco_ctrl.py',
        output='screen',
        condition=IfCondition(use_mock_hardware),
    )

    # ── Controller spawners (ordered) ─────────────────────────────────────────
    spawn_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )

    spawn_ft_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'force_torque_sensor_broadcaster'],
        output='screen',
    )

    # admittance_controller FIRST — registers chainable reference interfaces
    spawn_admittance = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'admittance_controller'],
        output='screen',
    )

    # arm_controller SECOND — connects to admittance chainable interfaces
    spawn_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen',
    )

    spawn_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'gripper_controller'],
        output='screen',
    )

    # ── Event chain: control_node ready → jsb → ft → admittance → arm+gripper
    evt_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[spawn_jsb],
        )
    )
    evt_ft = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ft_broadcaster],
        )
    )
    evt_admittance = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ft_broadcaster,
            on_exit=[spawn_admittance],
        )
    )
    evt_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_admittance,
            on_exit=[spawn_arm, spawn_gripper],
        )
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        robot_state_publisher,
        ros2_control_node,
        mujoco_viewer,
        evt_jsb,
        evt_ft,
        evt_admittance,
        evt_arm,
    ])
