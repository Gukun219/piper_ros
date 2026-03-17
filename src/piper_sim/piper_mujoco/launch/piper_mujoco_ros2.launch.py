"""
piper_mujoco_ros2.launch.py
────────────────────────────
Launch file for Piper arm simulation with ros2_control admittance control.

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
  use_mock_hardware  [true]  — Use mock_components/GenericSystem (default).
                    [false]  — Use mujoco_ros2_control/MuJoCoSystem.

Controller load order:
  1. joint_state_broadcaster
  2. force_torque_sensor_broadcaster
  3. admittance_controller + gripper_controller

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
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node


def remove_comments(text):
    """Strip XML comments (some parsers choke on them in robot_description)."""
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def generate_launch_description():
    pkg_description = get_package_share_directory('piper_description')
    pkg_mujoco      = get_package_share_directory('piper_mujoco')

    # breakpoint()

    # ── Launch argument ───────────────────────────────────────────────────────
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description=(
            'Use mock_components/GenericSystem (true) or '
            'mujoco_ros2_control/MuJoCoSystem (false).'
        ),
    )
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')

    # ── Robot description ─────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_description, 'urdf',
                              'piper_description_mujoco.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_mock_hardware': 'true',
                                      'lock_joints_4_6': 'true'})
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

    # ── MuJoCo visualization node (both modes) ───────────────────────────────
    # viewer_only=true  when use_mock_hardware=false (mujoco_ros2_control runs physics)
    # viewer_only=false when use_mock_hardware=true  (viewer drives physics itself)
    mujoco_viewer = Node(
        package='piper_mujoco',
        executable='piper_mujoco_ctrl.py',
        # executable='piper_mujoco_ctrl_debug.py',
        output='screen',
        parameters=[{'viewer_only': NotSubstitution(use_mock_hardware)}],
    )

    # ── Trajectory bridge (FollowJointTrajectory action → admittance topic) ──
    trajectory_bridge = Node(
        package='piper_mujoco',
        executable='admittance_trajectory_bridge.py',
        output='screen',
    )

    # ── World-frame force bridge (world Wrench → sensor-frame injectors) ──
    world_force_bridge = Node(
        package='piper_mujoco',
        executable='world_force_bridge.py',
        output='screen',
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
        robot_state_publisher,
        ros2_control_node,
        mujoco_viewer,
        trajectory_bridge,
        world_force_bridge,
        spawn_jsb,
        evt_ft,
        evt_admittance_and_gripper,
    ])
