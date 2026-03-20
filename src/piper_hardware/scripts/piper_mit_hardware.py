#!/usr/bin/env python3
"""
piper_mit_hardware.py
─────────────────────
Real-hardware bridge node: translates ros2_control position+velocity references
into piper_sdk JointMitCtrl calls.  No control computation is performed here —
the MIT PD law runs inside the robot firmware.

Control data-flow:
  admittance_trajectory_bridge.py
      │ /admittance_controller/joint_references (JointTrajectoryPoint)
      ▼
  AdmittanceController (ros2_control, optional — runs in mock-hw mode)
      │ modifies pos+vel based on FT sensor reading
      ▼
  [mock hardware state = commanded values in mock mode]
      │
  piper_mit_hardware.py  ← subscribes to /admittance_controller/joint_references
      │                     (or to a dedicated command topic, see cmd_topic param)
      │  interface conversion only — no control calculation
      ▼
  piper_sdk.JointMitCtrl(joint_id, pos_ref, vel_ref, kp, kd, t_ref=0)
      │
      ▼  (piper SDK reads encoder feedback)
  /piper_hardware/joint_states  (sensor_msgs/JointState)

Note on admittance compliance:
  This node subscribes to the *reference* topic (input to AdmittanceController),
  not to the controller's output hardware interfaces.  Full closed-loop admittance
  control on real hardware requires a C++ ros2_control hardware plugin.  This
  bridge provides basic trajectory following and is the first step for hardware
  bring-up.

Parameters (ROS2):
  cmd_topic   (str)   topic carrying position+velocity commands
                      default: /admittance_controller/joint_references
  joint_names (list)  joints to control in order
                      default: [joint1, joint2, joint3, joint5]
  kp          (list)  position gain per joint  (default: 10.0 each)
  kd          (list)  velocity gain per joint  (default: 0.8 each)
  publish_rate (float) Hz for joint_states republishing (default: 100.0)
  can_interface (str) CAN interface name passed to piper_sdk (default: can0)
"""

import sys

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

# piper_sdk import — optional at import time so the node can start without
# hardware attached (useful for dry-run / log inspection).
try:
    from piper_sdk import C_PiperInterface  # type: ignore[import]
    _SDK_AVAILABLE = True
except ImportError:
    _SDK_AVAILABLE = False

DEFAULT_JOINTS = ['joint1', 'joint2', 'joint3', 'joint5']
# SDK joint IDs are 1-based; the mapping below converts JOINT_NAMES index → SDK id.
# Adjust if the physical joint numbering differs from the logical order above.
SDK_JOINT_ID = {
    'joint1': 1,
    'joint2': 2,
    'joint3': 3,
    'joint4': 4,
    'joint5': 5,
    'joint6': 6,
}


class PiperMitHardware(Node):
    """
    ROS2 node: bridges trajectory references → piper_sdk JointMitCtrl calls.
    """

    def __init__(self):
        super().__init__('piper_mit_hardware')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('cmd_topic',
                               '/admittance_controller/joint_references')
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('kp', [10.0] * len(DEFAULT_JOINTS))
        self.declare_parameter('kd', [0.8] * len(DEFAULT_JOINTS))
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('dry_run', not _SDK_AVAILABLE)

        cmd_topic    = self.get_parameter('cmd_topic').value
        self._joints = self.get_parameter('joint_names').value
        self._kp     = self.get_parameter('kp').value
        self._kd     = self.get_parameter('kd').value
        rate         = self.get_parameter('publish_rate').value
        can_iface    = self.get_parameter('can_interface').value
        self._dry_run = self.get_parameter('dry_run').value

        n = len(self._joints)
        if len(self._kp) != n:
            self._kp = [self._kp[0]] * n if self._kp else [10.0] * n
        if len(self._kd) != n:
            self._kd = [self._kd[0]] * n if self._kd else [0.8] * n

        # ── piper SDK init ────────────────────────────────────────────────────
        self._piper = None
        if self._dry_run:
            self.get_logger().warn(
                'dry_run=True — piper_sdk calls are SKIPPED. '
                'Commands will be logged only.'
            )
        elif not _SDK_AVAILABLE:
            self.get_logger().error(
                'piper_sdk not found and dry_run=False. '
                'Install piper_sdk or set dry_run:=true.'
            )
            sys.exit(1)
        else:
            self._piper = C_PiperInterface()
            self._piper.ConnectPort(can_iface, 1000000)
            self._piper.EnableArm(7)
            self.get_logger().info(f'piper_sdk connected on {can_iface}')

        # ── State ─────────────────────────────────────────────────────────────
        self._last_cmd_pos = [0.0] * n
        self._last_cmd_vel = [0.0] * n

        # ── Publisher — joint states from hardware feedback ───────────────────
        self._js_pub = self.create_publisher(
            JointState,
            '/piper_hardware/joint_states',
            10,
        )

        # ── Subscriber — command references ───────────────────────────────────
        self.create_subscription(
            JointTrajectoryPoint,
            cmd_topic,
            self._cmd_cb,
            10,
        )

        # ── Feedback timer ────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._feedback_loop)

        self.get_logger().info(
            f'piper_mit_hardware ready  joints={self._joints}  '
            f'kp={self._kp}  kd={self._kd}  topic={cmd_topic}'
        )

    # ── Command callback ──────────────────────────────────────────────────────
    def _cmd_cb(self, msg: JointTrajectoryPoint):
        """Receive pos+vel reference and forward to piper SDK (interface conversion only)."""
        positions  = list(msg.positions)
        velocities = list(msg.velocities) if msg.velocities else [0.0] * len(self._joints)

        if len(positions) != len(self._joints):
            self.get_logger().warn(
                f'Expected {len(self._joints)} positions, got {len(positions)} — skipping'
            )
            return

        self._last_cmd_pos = positions
        self._last_cmd_vel = velocities

        for i, name in enumerate(self._joints):
            joint_id = SDK_JOINT_ID.get(name)
            if joint_id is None:
                self.get_logger().warn(f'No SDK joint ID for {name}')
                continue

            pos_rad = positions[i]
            vel_rad = velocities[i]
            kp = self._kp[i]
            kd = self._kd[i]

            if self._dry_run:
                self.get_logger().debug(
                    f'[dry_run] JointMitCtrl(id={joint_id}, '
                    f'pos={pos_rad:.4f}, vel={vel_rad:.4f}, '
                    f'kp={kp}, kd={kd}, t=0)'
                )
            else:
                # Pure interface conversion — no control calculation here.
                # The MIT PD law (τ = kp*(pos_ref-pos) + kd*(vel_ref-vel)) runs
                # inside the robot firmware using the values we supply.
                self._piper.JointMitCtrl(
                    joint_id,
                    pos_rad,   # pos_ref [rad]
                    vel_rad,   # vel_ref [rad/s]
                    kp,        # position gain
                    kd,        # velocity gain
                    0.0,       # t_ref = 0 (no feed-forward torque)
                )

    # ── Feedback loop — read piper SDK and republish joint states ─────────────
    def _feedback_loop(self):
        now = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = now
        msg.name = list(self._joints)

        if self._dry_run or self._piper is None:
            # In dry-run mode, echo commanded values as simulated feedback
            msg.position = list(self._last_cmd_pos)
            msg.velocity = list(self._last_cmd_vel)
            msg.effort   = [0.0] * len(self._joints)
        else:
            positions  = []
            velocities = []
            efforts    = []
            for name in self._joints:
                joint_id = SDK_JOINT_ID.get(name, 1)
                try:
                    fb = self._piper.GetArmJointFeedback()
                    # C_PiperInterface returns a feedback object; index by joint_id-1
                    positions.append(fb.joint_state[joint_id - 1].pos)
                    velocities.append(fb.joint_state[joint_id - 1].vel)
                    efforts.append(fb.joint_state[joint_id - 1].effort)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f'Feedback read error for {name}: {e}')
                    positions.append(0.0)
                    velocities.append(0.0)
                    efforts.append(0.0)
            msg.position = positions
            msg.velocity = velocities
            msg.effort   = efforts

        self._js_pub.publish(msg)

    def destroy_node(self):
        if self._piper is not None:
            try:
                self._piper.DisableArm(7)
                self.get_logger().info('piper_sdk arm disabled')
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PiperMitHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
