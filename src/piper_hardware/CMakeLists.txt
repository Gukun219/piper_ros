cmake_minimum_required(VERSION 3.5)
project(piper_hardware)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/piper_hardware_base.py
  scripts/piper_mit_hardware.py
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
