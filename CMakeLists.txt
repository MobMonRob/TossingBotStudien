cmake_minimum_required(VERSION 3.8)
project(ur5_rg2_ign)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf launch config ur5_rg2
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/pick_ball.py
  scripts/test_gripper.py
  scripts/estimate_inertial_properties.py
  scripts/ur5e_base_controller.py
  scripts/wurf_seite_positionsgesteuert.py
  scripts/wurf_seite_zeitgesteuert.py
  scripts/wurf_ueberkopf_zeitgesteuert.py
  scripts/wurf_unterarm_zeitgesteuert.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
