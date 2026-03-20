cmake_minimum_required(VERSION 3.5)
project(compliance_control)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/admittance_trajectory_bridge.py
  scripts/world_force_bridge.py
  scripts/admittance_demo.py
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

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_trajectory_following.py
    TIMEOUT 60)
  add_launch_test(test/test_admittance_compliance.py
    TIMEOUT 60)
endif()

ament_package()
