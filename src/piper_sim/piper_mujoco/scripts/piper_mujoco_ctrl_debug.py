#!/usr/bin/env python3
# coding=utf-8
"""
Debug wrapper for piper_mujoco_ctrl.
Run this instead of piper_mujoco_ctrl.py when you need breakpoint debugging.

Usage:
  ros2 run piper_mujoco piper_mujoco_ctrl_debug.py

Then attach VS Code debugger on port 5678.
"""

import debugpy
debugpy.listen(5678)
print("Waiting for debugger to attach on port 5678...")
debugpy.wait_for_client()

from piper_mujoco_ctrl import main
main()
