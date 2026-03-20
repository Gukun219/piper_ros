cmake_minimum_required(VERSION 3.5)
project(piper_mujoco)

find_package(ament_cmake REQUIRED)

# Install Python scripts (MuJoCo viewer nodes only)
install(PROGRAMS
  scripts/piper_mujoco_ctrl.py
  scripts/piper_mujoco_ctrl_debug.py
  scripts/piper_no_gripper_mujoco_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
