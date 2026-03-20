"""
piper_mit_control.launch.py
────────────────────────────
Launch file for Piper arm real-hardware MIT control mode.

Architecture:
  MoveIt / user goal
      │  FollowJointTrajectory action
      ▼
  admittance_trajectory_bridge.py  (trajectory interpolation)
      │  /admittance_controller/joint_references  (JointTrajectoryPoint: pos+vel)
      ▼
  AdmittanceController (ros2_control, mock hardware)
      │  modifies pos+vel based on FT sensor reading
      │  writes to mock hardware command interfaces (pos+vel)
      │
  piper_mit_hardware.py  ← also subscribes to joint_references directly
      │  pure interface conversion — no control calculation
      ▼
  piper_sdk.JointMitCtrl(pos_ref, vel_ref, kp, kd, t_ref=0)
      (MIT PD law runs inside robot firmware)

Note: mock_components/GenericSystem is used as the ros2_control hardware
plugin so the admittance controller stack runs without a C++ hardware plugin.
piper_mit_hardware.py connects to piper_sdk independently for actual hardware
control.  For full closed-loop admittance compliance on real hardware, a
dedicated C++ ros2_control hardware plugin is the recommended next step.

Launch arguments:
  kp           [10.0,10.0,10.0,10.0]  position gains per joint
  kd           [0.8,0.8,0.8,0.8]      velocity gains per joint
  can_interface [can0]                 CAN interface for piper_sdk
  dry_run      [false]                 skip piper_sdk calls (test mode)
"""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def remove_comments(text):
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


def generate_launch_description():
    pkg_description = get_package_share_directory('piper_description')
    pkg_hardware    = get_package_share_directory('piper_hardware')

    # ── Launch arguments ──────────────────────────────────────────────────────
    kp_arg = DeclareLaunchArgument(
        'kp', default_value='[10.0, 10.0, 10.0, 10.0]',
        description='Position gain per joint for JointMitCtrl',
    )
    kd_arg = DeclareLaunchArgument(
        'kd', default_value='[0.8, 0.8, 0.8, 0.8]',
        description='Velocity gain per joint for JointMitCtrl',
    )
    can_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='CAN interface name for piper_sdk',
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='If true, skip piper_sdk calls (log-only mode for testing)',
    )

    # ── Robot description — use mujoco xacro with mock hardware ──────────────
    # The piper_description_mujoco.xacro provides the FT sensor link and
    # ros2_control block needed by the admittance controller.
    # use_mock_hardware=true → mock_components/GenericSystem (no MuJoCo required)
    xacro_file = os.path.join(pkg_description, 'urdf',
                              'piper_description_mujoco.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        'use_mock_hardware': 'true',
        'lock_joints_4_6': 'true',
    })
    robot_description = remove_comments(doc.toxml())

    controllers_yaml = os.path.join(pkg_hardware, 'config', 'ros2_controllers.yaml')

    # ── 1. robot_state_publisher ──────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 50.0},
        ],
    )

    # ── 2. ros2_control_node (mock hardware — runs admittance controller) ─────
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # ── 3. piper_mit_hardware.py (real hardware bridge via piper_sdk) ─────────
    piper_mit_hardware = Node(
        package='piper_hardware',
        executable='piper_mit_hardware.py',
        output='screen',
        parameters=[{
            'cmd_topic':     '/admittance_controller/joint_references',
            'joint_names':   ['joint1', 'joint2', 'joint3', 'joint5'],
            'kp':            LaunchConfiguration('kp'),
            'kd':            LaunchConfiguration('kd'),
            'publish_rate':  100.0,
            'can_interface': LaunchConfiguration('can_interface'),
            'dry_run':       LaunchConfiguration('dry_run'),
        }],
    )

    # ── Controller spawners ───────────────────────────────────────────────────
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

    # ── 6. Trajectory bridge (FollowJointTrajectory → admittance topic) ───────
    trajectory_bridge = Node(
        package='piper_mujoco',
        executable='admittance_trajectory_bridge.py',
        output='screen',
    )

    # ── Event chain: jsb → ft_broadcaster → admittance + gripper ─────────────
    evt_ft = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ft_broadcaster],
        )
    )
    evt_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ft_broadcaster,
            on_exit=[spawn_admittance, spawn_gripper],
        )
    )

    return LaunchDescription([
        kp_arg,
        kd_arg,
        can_arg,
        dry_run_arg,
        robot_state_publisher,
        ros2_control_node,
        piper_mit_hardware,
        trajectory_bridge,
        spawn_jsb,
        evt_ft,
        evt_controllers,
    ])
