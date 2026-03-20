#!/usr/bin/env python3
"""
piper_hardware_base.py
──────────────────────
Abstract base class that defines the hardware interface contract for Piper arm
backends (real hardware, simulation stub, etc.).

All concrete implementations must supply:
  connect()            — open SDK / serial connection
  disconnect()         — release connection
  send_joint_commands() — write position+velocity to hardware
  get_joint_states()   — read position, velocity, effort from hardware

The MIT control law is executed **inside the physical robot** firmware.
The bridge layer only performs format conversion:

  AdmittanceController output: [pos_cmd, vel_cmd]
      ↓ interface conversion (no control calculation)
  JointMitCtrl(pos_ref=pos_cmd, vel_ref=vel_cmd, kp, kd, t_ref=0)
"""

import abc
from dataclasses import dataclass, field
from typing import List


@dataclass
class JointState:
    """Snapshot of joint feedback from hardware."""
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    efforts: List[float] = field(default_factory=list)


class PiperHardwareBase(abc.ABC):
    """Abstract hardware backend for Piper arm."""

    @abc.abstractmethod
    def connect(self) -> None:
        """Open hardware connection (SDK init, serial port, etc.)."""

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Release hardware connection and stop all motion."""

    @abc.abstractmethod
    def send_joint_commands(
        self,
        positions: List[float],
        velocities: List[float],
    ) -> None:
        """
        Forward position+velocity references to hardware.

        Parameters
        ----------
        positions  : desired joint angles [rad], length = num_joints
        velocities : desired joint velocities [rad/s], length = num_joints

        Note: No control law is computed here — the values are passed directly
        to the hardware MIT interface (JointMitCtrl with t_ref=0).
        """

    @abc.abstractmethod
    def get_joint_states(self) -> JointState:
        """
        Read current joint state from hardware.

        Returns
        -------
        JointState with positions [rad], velocities [rad/s], efforts [N·m]
        """

    @property
    @abc.abstractmethod
    def num_joints(self) -> int:
        """Number of controlled joints."""

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        """True if the hardware connection is active."""
