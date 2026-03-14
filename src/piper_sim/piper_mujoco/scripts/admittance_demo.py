#!/usr/bin/env python3
"""
Admittance controller demo — inject sinusoidal force and observe compliance.

Usage:
  1. Launch the simulation:
       ros2 launch piper_mujoco piper_mujoco_ros2.launch.py

  2. In another terminal, run this demo:
       ros2 run piper_mujoco admittance_demo.py

  3. Watch the MuJoCo viewer: the arm holds a position, then a sinusoidal
     force is applied along Z. The admittance controller causes the arm
     to deviate from the reference in response to the force.

  4. Monitor the FT sensor readings:
       ros2 topic echo /ft_sensor_broadcaster/wrench
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration


class AdmittanceDemo(Node):
    def __init__(self):
        super().__init__('admittance_demo')

        # Action client for trajectory
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        # Publishers for FT force injection
        self._ft_fx_pub = self.create_publisher(
            Float64MultiArray, '/ft_fx_injector/commands', 10)
        self._ft_fy_pub = self.create_publisher(
            Float64MultiArray, '/ft_fy_injector/commands', 10)
        self._ft_fz_pub = self.create_publisher(
            Float64MultiArray, '/ft_fz_injector/commands', 10)

    def send_trajectory(self, positions, duration_sec=3.0):
        """Send arm to target position via FollowJointTrajectory action."""
        self.get_logger().info('Waiting for action server...')
        self._traj_client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint5'
        ]
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9),
        )
        goal.trajectory.points = [pt]

        self.get_logger().info(
            f'Sending trajectory: {positions} in {duration_sec}s'
        )
        future = self._traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Trajectory completed')
        return True

    def inject_force(self, fx=0.0, fy=0.0, fz=0.0):
        """Set FT sensor force values via forward_command_controllers."""
        msg_x = Float64MultiArray()
        msg_x.data = [fx]
        self._ft_fx_pub.publish(msg_x)

        msg_y = Float64MultiArray()
        msg_y.data = [fy]
        self._ft_fy_pub.publish(msg_y)

        msg_z = Float64MultiArray()
        msg_z.data = [fz]
        self._ft_fz_pub.publish(msg_z)

    def run_demo(self):
        self.get_logger().info('=== Admittance Controller Demo ===')

        # Phase 1: Move arm to a demo position
        self.get_logger().info('[Phase 1] Moving arm to demo position...')
        target = [0.0, 0.5, -0.3, 0.5]
        if not self.send_trajectory(target, duration_sec=3.0):
            return

        # Phase 2: Hold position with zero force (2 seconds)
        self.get_logger().info(
            '[Phase 2] Holding position with zero force (2s)...'
        )
        for _ in range(20):
            self.inject_force(0.0, 0.0, 0.0)
            time.sleep(0.1)

        # Phase 3: Apply sinusoidal force along Z axis
        self.get_logger().info(
            '[Phase 3] Applying sinusoidal force along Z (10s)...'
        )
        self.get_logger().info(
            '  Watch the arm deviate from reference due to admittance!'
        )
        self.get_logger().info(
            '  Monitor: ros2 topic echo /ft_sensor_broadcaster/wrench'
        )

        duration = 10.0   # seconds
        freq = 0.5        # Hz
        amplitude = 20.0  # Newtons
        rate = 100         # Hz publish rate
        t0 = time.time()

        while time.time() - t0 < duration:
            t = time.time() - t0
            fz = amplitude * math.sin(2.0 * math.pi * freq * t)
            self.inject_force(0.0, 0.0, fz)
            time.sleep(1.0 / rate)

        # Phase 4: Remove force
        self.get_logger().info('[Phase 4] Removing force — arm returns...')
        for _ in range(30):
            self.inject_force(0.0, 0.0, 0.0)
            time.sleep(0.1)

        self.get_logger().info('=== Demo complete ===')


def main():
    rclpy.init()
    demo = AdmittanceDemo()
    try:
        demo.run_demo()
    except KeyboardInterrupt:
        demo.inject_force(0.0, 0.0, 0.0)
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
