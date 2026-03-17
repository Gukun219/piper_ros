cmake_minimum_required(VERSION 3.5)
project(piper_mujoco)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/piper_mujoco_ctrl.py
  scripts/piper_mujoco_ctrl_debug.py
  scripts/piper_no_gripper_mujoco_ctrl.py
  scripts/admittance_trajectory_bridge.py
  scripts/admittance_demo.py
  scripts/world_force_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
