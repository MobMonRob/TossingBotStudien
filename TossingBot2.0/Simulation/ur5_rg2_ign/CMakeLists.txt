cmake_minimum_required(VERSION 3.0.2)
project(ur5_rg2_ign)

# Find dependencies
find_package(catkin REQUIRED)

# Declare catkin package
catkin_package()

# Install directories
install(DIRECTORY urdf ur5_rg2/meshes launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Make scripts executable
catkin_install_python(PROGRAMS
  scripts/test_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
