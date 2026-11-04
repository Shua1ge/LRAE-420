cmake_minimum_required(VERSION 3.0.2)
project(lrae_go2)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mid360_to_world.py
  scripts/stuck_detector.py
  scripts/startup_spin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
