#!/usr/bin/env python3
"""
Integration test: trajectory tracking accuracy.

Verifies the full chain:
  FollowJointTrajectory action → trajectory_bridge
  → admittance_controller → mock hardware
  → joint_states feedback

Uses mock hardware (no MuJoCo viewer, CI-friendly).
"""

import os
import re
import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from rclpy.action import ActionClient
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint5']

# Target positions for the trajectory goal
TARGET_POSITIONS = {
    'joint1': 0.3,
    'joint2': 0.2,
    'joint3': -0.1,
    'joint5': -0.2,
}

TRAJECTORY_DURATION_SEC = 3


def _remove_comments(text):
    return re.sub(r'<!--.*?-->', '', text, flags=re.DOTALL)


@pytest.mark.launch_test
def generate_test_description():
    show_viewer = os.environ.get('PIPER_VISUAL_TEST', '').lower() in ('1', 'true')

    pkg_description = get_package_share_directory('piper_description')
    pkg_compliance = get_package_share_directory('compliance_control')

    xacro_file = os.path.join(pkg_description, 'urdf', 'piper_description_mujoco.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={'use_mock_hardware': 'true',
                                     'lock_joints_4_6': 'true'})
    robot_description = _remove_comments(doc.toxml())

    controllers_yaml = os.path.join(pkg_compliance, 'config', 'ros2_controllers.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 50.0},
        ],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        remappings=[('~/robot_description', '/robot_description')],
    )

    trajectory_bridge = Node(
        package='compliance_control',
        executable='admittance_trajectory_bridge.py',
        output='screen',
    )

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

    mujoco_viewer = Node(
        package='piper_mujoco',
        executable='piper_mujoco_ctrl.py',
        parameters=[{'viewer_only': False}],
        output='screen',
    ) if show_viewer else None

    ready_to_test = (
        TimerAction(period=5.0, actions=[launch_testing.actions.ReadyToTest()])
        if show_viewer
        else launch_testing.actions.ReadyToTest()
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        trajectory_bridge,
        spawn_jsb,
        evt_ft,
        evt_controllers,
        ready_to_test,
    ]
    if mujoco_viewer:
        nodes.insert(2, mujoco_viewer)
    return launch.LaunchDescription(nodes)


class _TestNode(RclpyNode):
    """Helper node for sending goals and reading joint states."""

    def __init__(self):
        super().__init__('test_trajectory_helper')
        self._latest_positions = {}
        self._lock = threading.Lock()

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_positions[name] = pos

    def get_positions(self):
        with self._lock:
            return dict(self._latest_positions)

    def wait_for_action_server(self, timeout_sec=25.0):
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal_sync(self, positions_dict, duration_sec, timeout_sec=15.0):
        """Send a FollowJointTrajectory goal and wait for result."""
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = [positions_dict[j] for j in JOINT_NAMES]
        pt.velocities = [0.0] * len(JOINT_NAMES)
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        traj.points = [pt]
        goal.trajectory = traj

        send_future = self._action_client.send_goal_async(goal)

        # Wait for goal acceptance
        deadline = time.time() + timeout_sec
        while not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                return None, 'timeout waiting for goal acceptance'

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return None, 'goal rejected'

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > deadline:
                return None, 'timeout waiting for result'

        return result_future.result(), None


class TestTrajectoryFollowing(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = _TestNode()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_trajectory_goal_success(self):
        # Wait for action server (controllers must be spawned and active)
        ready = self.node.wait_for_action_server(timeout_sec=25.0)
        self.assertTrue(ready, 'Action server did not become available within 25 s')

        # Send goal
        result_obj, err = self.node.send_goal_sync(
            TARGET_POSITIONS,
            duration_sec=TRAJECTORY_DURATION_SEC,
            timeout_sec=15.0,
        )
        self.assertIsNotNone(result_obj, f'Goal failed: {err}')

        # result.result is FollowJointTrajectory.Result; error_code 0 = SUCCESSFUL
        error_code = result_obj.result.error_code
        self.assertEqual(
            error_code, 0,
            f'Action result error_code={error_code} (expected 0 = SUCCESSFUL)',
        )

        # Settle briefly, then read final joint positions
        time.sleep(0.5)
        rclpy.spin_once(self.node, timeout_sec=0.3)
        positions = self.node.get_positions()

        for joint, target in TARGET_POSITIONS.items():
            actual = positions.get(joint)
            self.assertIsNotNone(actual, f'No position reported for {joint}')
            err = abs(actual - target)
            self.assertLess(
                err, 0.05,
                f'{joint}: actual={actual:.4f} target={target:.4f} error={err:.4f} rad '
                f'(limit 0.05 rad)',
            )
