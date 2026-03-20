#!/usr/bin/env python3
"""
World-frame force injection bridge for mock hardware.

Subscribes to /world_force_cmd (geometry_msgs/Wrench) where forces/torques
are specified in the world frame, transforms them into the ft_sensor_link
frame using TF, and publishes to the ft_f{x,y,z}_injector/commands topics.

Usage:
  # In one terminal, launch the simulation:
  ros2 launch piper_mujoco piper_mujoco_ros2.launch.py

  # In another terminal, inject a 20N force along world Z (up):
  ros2 topic pub /world_force_cmd geometry_msgs/Wrench \
    "{force: {x: 0, y: 0, z: 20}, torque: {x: 0, y: 0, z: 0}}"

  # Or use a one-liner script for sinusoidal force:
  python3 -c "
  import rclpy, math, time
  from rclpy.node import Node
  from geometry_msgs.msg import Wrench
  rclpy.init()
  node = Node('force_pub')
  pub = node.create_publisher(Wrench, '/world_force_cmd', 10)
  t0 = time.time()
  while rclpy.ok():
      F = 20.0 * math.sin(2*math.pi*0.5*(time.time()-t0))
      msg = Wrench()
      msg.force.z = F  # world Z = up
      pub.publish(msg)
      time.sleep(0.01)
  "
"""

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener


class WorldForceBridge(Node):
    def __init__(self):
        super().__init__('world_force_bridge')

        # TF buffer to look up world -> ft_sensor_link transform
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Subscriber: world-frame wrench command
        self.create_subscription(
            Wrench, '/world_force_cmd', self._on_wrench, 10)

        # Publishers: sensor-frame force/torque injectors
        self._pubs = {
            'fx': self.create_publisher(
                Float64MultiArray, '/ft_fx_injector/commands', 10),
            'fy': self.create_publisher(
                Float64MultiArray, '/ft_fy_injector/commands', 10),
            'fz': self.create_publisher(
                Float64MultiArray, '/ft_fz_injector/commands', 10),
            'tx': self.create_publisher(
                Float64MultiArray, '/ft_tx_injector/commands', 10),
            'ty': self.create_publisher(
                Float64MultiArray, '/ft_ty_injector/commands', 10),
            'tz': self.create_publisher(
                Float64MultiArray, '/ft_tz_injector/commands', 10),
        }

        self.get_logger().info(
            'World force bridge ready. '
            'Publish geometry_msgs/Wrench to /world_force_cmd')

    @staticmethod
    def _quat_to_rotation_matrix(q):
        """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    def _on_wrench(self, msg: Wrench):
        """Transform world-frame wrench to sensor frame and publish."""
        try:
            tf = self._tf_buffer.lookup_transform(
                'ft_sensor_link', 'world', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        q = tf.transform.rotation
        # R_sensor_world: rotates vectors from world frame to sensor frame
        R = self._quat_to_rotation_matrix([q.x, q.y, q.z, q.w])

        # Rotate force and torque from world to sensor frame
        f_world = np.array([msg.force.x, msg.force.y, msg.force.z])
        t_world = np.array([msg.torque.x, msg.torque.y, msg.torque.z])

        f_sensor = R @ f_world
        t_sensor = R @ t_world

        # Publish to individual injector topics
        for key, val in [('fx', f_sensor[0]), ('fy', f_sensor[1]),
                         ('fz', f_sensor[2]), ('tx', t_sensor[0]),
                         ('ty', t_sensor[1]), ('tz', t_sensor[2])]:
            out = Float64MultiArray()
            out.data = [float(val)]
            self._pubs[key].publish(out)


def main():
    rclpy.init()
    node = WorldForceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
