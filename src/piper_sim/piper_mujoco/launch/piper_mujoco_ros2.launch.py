"""
piper_mujoco_ros2.launch.py
────────────────────────────
Launch file for Piper arm simulation with MuJoCo + ros2_control.

Controller chain:
  MoveIt  ──(FollowJointTrajectory action)──►  arm_controller (JTC, chainable)
                                                     │ chainable reference pos
                                                     ▼
                                             admittance_controller
                                                     │ position command
                                                     ▼
                                         mujoco_ros2_control hardware interface
                                                     │
                                                     ▼
                                            MuJoCo physics engine

Prerequisites:
  sudo apt install ros-humble-mujoco-ros2-control \\
                   ros-humble-admittance-controller \\
                   ros-humble-force-torque-sensor-broadcaster \\
                   ros-humble-kinematics-interface \\
                   ros-humble-kinematics-interface-kdl

Controller load order (important for chaining):
  1. joint_state_broadcaster
  2. force_torque_sensor_broadcaster
  3. admittance_controller   ← registers chainable interfaces
  4. arm_controller          ← connects to admittance chainable interfaces
  5. gripper_controller
"""

import os
import re
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def remove_comments(text):
    """Strip XML comments (some parsers choke on them in robot_description)."""
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def generate_launch_description():
    pkg_description = get_package_share_directory('piper_description')
    pkg_mujoco     = get_package_share_directory('piper_mujoco')

    # ── Robot description (URDF with MuJoCo ros2_control block) ──────────────
    xacro_file = os.path.join(pkg_description, 'urdf',
                              'piper_description_mujoco.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = remove_comments(doc.toxml())

    controllers_yaml = os.path.join(pkg_mujoco, 'config', 'ros2_controllers.yaml')

    # ── robot_state_publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 50.0},
        ],
    )

    # ── ros2_control_node (mujoco_ros2_control provides this or we use the
    #    standard one; mujoco_ros2_control may launch MuJoCo viewer internally)
    # ─────────────────────────────────────────────────────────────────────────
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    # 1. Joint state broadcaster
    spawn_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen',
    )

    # 2. FT sensor broadcaster (publishes wrench topic for monitoring)
    spawn_ft_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'force_torque_sensor_broadcaster'],
        output='screen',
    )

    # 3. admittance_controller FIRST — registers chainable reference interfaces
    spawn_admittance = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'admittance_controller'],
        output='screen',
    )

    # 4. arm_controller SECOND — connects to admittance chainable interfaces
    spawn_arm = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen',
    )

    # 5. gripper_controller (independent, writes joint7/8 directly)
    spawn_gripper = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'gripper_controller'],
        output='screen',
    )

    # ── Launch sequence: ros2_control_node → jsb → ft → admittance → arm+gripper
    evt_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[spawn_jsb],
        )
    )

    evt_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ft_broadcaster],
        )
    )

    evt_after_ft = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ft_broadcaster,
            on_exit=[spawn_admittance],
        )
    )

    evt_after_admittance = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_admittance,
            on_exit=[spawn_arm, spawn_gripper],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        evt_after_control_node,
        evt_after_jsb,
        evt_after_ft,
        evt_after_admittance,
    ])
