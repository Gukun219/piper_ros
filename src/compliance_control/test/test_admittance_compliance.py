#!/usr/bin/env python3
"""
Integration test: admittance compliance (force → displacement).

Verifies that injecting an external force via ft_fz_injector causes
the admittance controller to shift joint positions (compliant response),
and that removing the force causes the joints to move back toward the
initial position.

Uses mock hardware (no MuJoCo viewer, CI-friendly).

Theoretical expectation (comment only, not asserted):
  Static Cartesian displacement ≈ F/K = 10 N / 200 N·m⁻¹ = 0.05 m
  Actual joint displacement depends on Jacobian at home position.
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
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint5']


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

    # FT injectors (mock hardware only)
    spawn_ft_fx = Node(package='controller_manager', executable='spawner',
                       arguments=['ft_fx_injector'], output='screen')
    spawn_ft_fy = Node(package='controller_manager', executable='spawner',
                       arguments=['ft_fy_injector'], output='screen')
    spawn_ft_fz = Node(package='controller_manager', executable='spawner',
                       arguments=['ft_fz_injector'], output='screen')
    spawn_ft_tx = Node(package='controller_manager', executable='spawner',
                       arguments=['ft_tx_injector'], output='screen')
    spawn_ft_ty = Node(package='controller_manager', executable='spawner',
                       arguments=['ft_ty_injector'], output='screen')
    spawn_ft_tz = Node(package='controller_manager', executable='spawner',
                       arguments=['ft_tz_injector'], output='screen')

    evt_ft = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ft_broadcaster],
        )
    )

    evt_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ft_broadcaster,
            on_exit=[
                spawn_admittance, spawn_gripper,
                spawn_ft_fx, spawn_ft_fy, spawn_ft_fz,
                spawn_ft_tx, spawn_ft_ty, spawn_ft_tz,
            ],
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


class _ComplianceTestNode(RclpyNode):
    """Helper node that reads joint_states and publishes FT force commands."""

    def __init__(self):
        super().__init__('test_compliance_helper')
        self._latest_positions = {}
        self._lock = threading.Lock()

        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        self._fz_pub = self.create_publisher(
            Float64MultiArray,
            '/ft_fz_injector/commands',
            10,
        )

    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._latest_positions[name] = pos

    def get_positions(self):
        with self._lock:
            return dict(self._latest_positions)

    def set_fz(self, force_n: float):
        msg = Float64MultiArray()
        msg.data = [force_n]
        self._fz_pub.publish(msg)

    def spin_for(self, duration_sec: float):
        """Spin and optionally publish force while waiting."""
        end = time.time() + duration_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_joint_states(self, timeout_sec=20.0):
        """Block until at least one joint state message is received."""
        end = time.time() + timeout_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._latest_positions:
                    return True
        return False


def _max_displacement(pos_a: dict, pos_b: dict) -> float:
    return max(abs(pos_b.get(j, 0.0) - pos_a.get(j, 0.0)) for j in JOINT_NAMES)


class TestAdmittanceCompliance(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = _ComplianceTestNode()

    @classmethod
    def tearDownClass(cls):
        # Make sure force is zeroed on cleanup
        cls.node.set_fz(0.0)
        cls.node.spin_for(0.2)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_force_induces_displacement(self):
        # Wait for joint_states to appear (controllers must be active)
        ready = self.node.wait_for_joint_states(timeout_sec=25.0)
        self.assertTrue(ready, 'No /joint_states received within 25 s')

        # Let the system settle at the home position
        self.node.spin_for(2.0)
        pos_initial = self.node.get_positions()
        self.assertGreater(len(pos_initial), 0, 'No joint positions available')

        # Inject 10 N in the Z direction and hold for 2 s
        # Publish at ~10 Hz while waiting so the controller receives the command
        end = time.time() + 2.0
        while time.time() < end:
            self.node.set_fz(10.0)
            self.node.spin_for(0.1)

        pos_final = self.node.get_positions()
        displacement = _max_displacement(pos_initial, pos_final)

        # At least one joint should have moved (compliant response)
        self.assertGreater(
            displacement, 0.005,
            f'No compliance response: max displacement={displacement:.5f} rad '
            f'(expected > 0.005 rad). '
            f'Initial: {pos_initial}, Final: {pos_final}',
        )

        # Displacement must be bounded (divergence / instability check).
        # Mock hardware mirrors commands to state without inertia, so the
        # admittance integrator can accumulate larger values than on real hw.
        # 2.0 rad is the joint range limit; anything beyond that indicates
        # controller divergence rather than normal compliance.
        self.assertLess(
            displacement, 2.0,
            f'Displacement too large: {displacement:.4f} rad (limit 2.0 rad, '
            f'suggests controller divergence)',
        )

    def test_displacement_recovers_on_force_removal(self):
        # --- Setup phase: inject force to get a displaced state ---
        ready = self.node.wait_for_joint_states(timeout_sec=25.0)
        self.assertTrue(ready, 'No /joint_states received within 25 s')

        self.node.spin_for(1.0)
        pos_initial = self.node.get_positions()

        # Apply force
        end = time.time() + 2.0
        while time.time() < end:
            self.node.set_fz(10.0)
            self.node.spin_for(0.1)

        pos_displaced = self.node.get_positions()
        displacement_under_force = _max_displacement(pos_initial, pos_displaced)

        # Skip recovery assertion if force caused negligible displacement
        # (this prevents a false failure when the compliance test ran first
        #  and the joints are still displaced)
        if displacement_under_force < 0.005:
            self.skipTest(
                f'Force caused negligible displacement ({displacement_under_force:.5f} rad); '
                'cannot meaningfully test recovery'
            )

        # --- Recovery phase: remove force and wait ---
        self.node.set_fz(0.0)
        self.node.spin_for(2.0)
        pos_recover = self.node.get_positions()

        displacement_after_removal = _max_displacement(pos_initial, pos_recover)

        # After removing force the joints should move back toward initial position
        self.assertLess(
            displacement_after_removal,
            displacement_under_force,
            f'Joints did not recover after force removal: '
            f'under_force={displacement_under_force:.4f} rad, '
            f'after_removal={displacement_after_removal:.4f} rad',
        )
