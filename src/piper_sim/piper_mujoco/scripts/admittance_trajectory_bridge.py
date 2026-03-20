#!/usr/bin/env python3
"""
Trajectory bridge for admittance_controller (Humble topic-based architecture).

Provides a FollowJointTrajectory action server at /arm_controller/follow_joint_trajectory.
Interpolates trajectory goals and publishes JointTrajectoryPoint references to
/admittance_controller/joint_references so that the admittance controller can apply
force-torque compliance before commanding the hardware.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint5']


def _to_sec(duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


class AdmittanceTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('admittance_trajectory_bridge')

        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('goal_tolerance', 0.02)

        publish_rate = self.get_parameter('publish_rate').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value

        self._lock = threading.Lock()
        self._current_positions = [0.0] * len(JOINT_NAMES)
        # Fixed reference position — NOT updated by joint_states feedback.
        # Only updated when a new trajectory goal completes or is set.
        # This prevents positive feedback: admittance shifts joints → bridge
        # reads shifted positions as new reference → drift.
        self._hold_position = [0.0] * len(JOINT_NAMES)
        self._hold_position_initialized = False
        self._trajectory_data = None  # set when goal is active
        self._active_goal_handle = None

        cb_group = ReentrantCallbackGroup()

        # Publisher → admittance reference
        self._ref_pub = self.create_publisher(
            JointTrajectoryPoint,
            '/admittance_controller/joint_references',
            10,
        )

        # Subscriber ← joint states (for feedback & goal-reached detection)
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
            callback_group=cb_group,
        )

        # Action server (same name MoveIt expects)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb_group,
        )

        # Control loop timer
        self._timer = self.create_timer(
            1.0 / publish_rate,
            self._control_loop,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'Trajectory bridge ready  (rate={publish_rate} Hz, '
            f'tol={self._goal_tolerance} rad)'
        )

    # ── Joint state subscriber ────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(JOINT_NAMES):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self._current_positions[i] = msg.position[idx]
            # Capture initial position once as the hold reference
            if not self._hold_position_initialized:
                self._hold_position = list(self._current_positions)
                self._hold_position_initialized = True

    # ── Action callbacks ──────────────────────────────────────────────────────
    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory

        # Build joint index mapping: trajectory joint order → JOINT_NAMES order
        try:
            idx_map = [traj.joint_names.index(name) for name in JOINT_NAMES]
        except ValueError as e:
            self.get_logger().error(f'Missing joint in goal: {e}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Reorder trajectory points to canonical JOINT_NAMES order
        reordered_points = []
        for pt in traj.points:
            rpt = JointTrajectoryPoint()
            rpt.positions = [pt.positions[j] for j in idx_map]
            if pt.velocities:
                rpt.velocities = [pt.velocities[j] for j in idx_map]
            rpt.time_from_start = pt.time_from_start
            reordered_points.append(rpt)

        # Prepend current position as t=0 start point (for interpolation)
        start_pt = JointTrajectoryPoint()
        with self._lock:
            start_pt.positions = list(self._current_positions)
        start_pt.velocities = [0.0] * len(JOINT_NAMES)
        start_pt.time_from_start.sec = 0
        start_pt.time_from_start.nanosec = 0

        all_points = [start_pt] + reordered_points
        total_time = _to_sec(all_points[-1].time_from_start)

        # Activate trajectory
        with self._lock:
            self._trajectory_data = {
                'points': all_points,
                'start_time': self.get_clock().now(),
                'total_time': total_time,
            }
            self._active_goal_handle = goal_handle

        self.get_logger().info(
            f'Executing trajectory: {len(traj.points)} points, '
            f'{total_time:.2f}s'
        )

        # Wait for completion
        feedback_msg = FollowJointTrajectory.Feedback()
        rate = self.create_rate(20)  # 20 Hz feedback

        while rclpy.ok():
            # Check cancel
            if goal_handle.is_cancel_requested:
                with self._lock:
                    # Hold at current interpolated position
                    if traj_data:
                        el = (self.get_clock().now() - traj_data['start_time']).nanoseconds * 1e-9
                        ref = self._interpolate(traj_data, el)
                        self._hold_position = list(ref.positions)
                    self._trajectory_data = None
                    self._active_goal_handle = None
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return FollowJointTrajectory.Result()

            with self._lock:
                traj_data = self._trajectory_data
                cur_pos = list(self._current_positions)

            # Trajectory finished (timer has already sent all points)
            if traj_data is None:
                break

            elapsed = (self.get_clock().now() - traj_data['start_time']).nanoseconds * 1e-9
            if elapsed >= traj_data['total_time']:
                # Check if joints reached final target
                final_pos = traj_data['points'][-1].positions
                errors = [abs(c - t) for c, t in zip(cur_pos, final_pos)]
                max_err = max(errors)

                if max_err < self._goal_tolerance:
                    with self._lock:
                        self._hold_position = list(final_pos)
                        self._trajectory_data = None
                        self._active_goal_handle = None
                    goal_handle.succeed()
                    self.get_logger().info(
                        f'Goal reached (max_err={max_err:.4f} rad)'
                    )
                    return FollowJointTrajectory.Result()

                # Still converging — keep waiting up to 2x total_time
                if elapsed > traj_data['total_time'] * 2.0 + 2.0:
                    with self._lock:
                        self._hold_position = list(final_pos)
                        self._trajectory_data = None
                        self._active_goal_handle = None
                    self.get_logger().warn(
                        f'Goal timeout (max_err={max_err:.4f} rad)'
                    )
                    goal_handle.abort()
                    return FollowJointTrajectory.Result()

            # Publish feedback
            feedback_msg.joint_names = JOINT_NAMES
            feedback_msg.actual.positions = cur_pos
            if traj_data:
                desired = self._interpolate(traj_data, elapsed)
                feedback_msg.desired.positions = desired.positions
                feedback_msg.error.positions = [
                    c - d for c, d in zip(cur_pos, desired.positions)
                ]
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()

        return FollowJointTrajectory.Result()

    # ── Interpolation ─────────────────────────────────────────────────────────
    def _interpolate(self, traj_data, elapsed: float) -> JointTrajectoryPoint:
        points = traj_data['points']

        # Clamp to last point
        if elapsed >= _to_sec(points[-1].time_from_start):
            return points[-1]

        # Find bracketing segment
        for i in range(len(points) - 1):
            t0 = _to_sec(points[i].time_from_start)
            t1 = _to_sec(points[i + 1].time_from_start)
            if t0 <= elapsed < t1:
                dt = (t1 - t0) if (t1 - t0) > 0 else 1.0
                alpha = (elapsed - t0) / dt
                msg = JointTrajectoryPoint()
                p0s = points[i].positions
                p1s = points[i + 1].positions
                msg.positions = [p0 + alpha * (p1 - p0) for p0, p1 in zip(p0s, p1s)]
                # Interpolate velocities if available; otherwise derive from position slope
                if points[i].velocities and points[i + 1].velocities:
                    v0s = points[i].velocities
                    v1s = points[i + 1].velocities
                    msg.velocities = [v0 + alpha * (v1 - v0) for v0, v1 in zip(v0s, v1s)]
                else:
                    msg.velocities = [(p1 - p0) / dt for p0, p1 in zip(p0s, p1s)]
                return msg

        return points[0]

    # ── Control loop (publishes reference at fixed rate) ──────────────────────
    def _control_loop(self):
        with self._lock:
            traj_data = self._trajectory_data
            hold_pos = list(self._hold_position)

        msg = JointTrajectoryPoint()

        if traj_data is not None:
            elapsed = (self.get_clock().now() - traj_data['start_time']).nanoseconds * 1e-9
            ref = self._interpolate(traj_data, elapsed)
            msg.positions = ref.positions
            msg.velocities = ref.velocities if ref.velocities else [0.0] * len(JOINT_NAMES)
        else:
            # Hold FIXED reference position (not current joint_states!)
            msg.positions = hold_pos
            msg.velocities = [0.0] * len(JOINT_NAMES)

        self._ref_pub.publish(msg)


def main():
    rclpy.init()
    node = AdmittanceTrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
