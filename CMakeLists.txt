cmake_minimum_required(VERSION 3.6.0)
project(sarc_gazebo_simulation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(
  DIRECTORY launch media models world_models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/fire.py scripts/forest.py scripts/forest2.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
