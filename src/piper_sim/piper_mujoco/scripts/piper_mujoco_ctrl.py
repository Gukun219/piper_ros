#!/usr/bin/env python3
# coding=utf-8

import sys
print(sys.executable)


import os
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


class MujocoModel(Node):
    def __init__(self):
        super().__init__("mujoco_joint_controller")
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.joint_targets = {}

        pkg_share_dir = get_package_share_directory('piper_description')
        model_path = os.path.join(pkg_share_dir, 'mujoco_model', 'piper_description.xml')
        model_path = os.path.abspath(model_path)

        self.get_logger().info(f"The model path is: {model_path}")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Build joint name → qpos index mapping
        self.joint_qpos_idx = {}
        for i in range(self.model.njnt):
            name = self.model.joint(i).name
            self.joint_qpos_idx[name] = self.model.joint(i).qposadr[0]

        # Build actuator name → ctrl index mapping
        self.actuator_idx = {}
        for i in range(self.model.nu):
            name = self.model.actuator(i).name
            self.actuator_idx[name] = i

        self.declare_parameter('viewer_only', False)
        self.viewer_only = self.get_parameter('viewer_only').value

        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz 控制循环
        self.tolerance = 0.05  # 角度误差容忍度

    def joint_state_callback(self, msg):
        """ 从 ROS 2 /joint_states 话题获取关节角度 """
        for i, name in enumerate(msg.name):
            self.joint_targets[name] = msg.position[i]

        # 确保 joint8 为 joint7 的负值
        if "joint7" in self.joint_targets:
            self.joint_targets["joint8"] = -self.joint_targets["joint7"]

    def pos_ctrl(self, joint_name, target_angle):
        """ 控制 MuJoCo 关节角度 """
        if joint_name not in self.actuator_idx:
            self.get_logger().warn(f"Joint {joint_name} not found in Mujoco model.")
            return
        try:
            actuator_id = self.actuator_idx[joint_name]
            self.data.ctrl[actuator_id] = target_angle
        except Exception as e:
            self.get_logger().error(f"Error controlling joint {joint_name}: {e}")

    def control_loop(self):
        """ 让 MuJoCo 机械臂跟随 ROS 关节状态 """
        if self.viewer_only:
            # viewer_only 模式：直接写 qpos，不跑物理（mujoco_ros2_control 已在插件内跑）
            for joint, angle in self.joint_targets.items():
                if joint in self.joint_qpos_idx:
                    self.data.qpos[self.joint_qpos_idx[joint]] = angle
            mujoco.mj_forward(self.model, self.data)
        else:
            # mock 模式：通过 actuator 驱动 + 物理步进
            for joint, target_angle in self.joint_targets.items():
                if joint in self.joint_qpos_idx:
                    self.pos_ctrl(joint, target_angle)
            mujoco.mj_step(self.model, self.data)
        if self.viewer.is_running():
            self.viewer.sync()


def main():
    rclpy.init()
    mujoco_node = MujocoModel()
    rclpy.spin(mujoco_node)
    mujoco_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
