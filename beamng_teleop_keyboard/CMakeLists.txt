cmake_minimum_required(VERSION 2.8.3)
project(beamng_teleop_keyboard)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)
catkin_python_setup()
catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs
)
catkin_install_python(PROGRAMS
  nodes/teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
