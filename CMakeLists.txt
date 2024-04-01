cmake_minimum_required(VERSION 2.8)
project(stero_manipulation)

find_package(catkin REQUIRED rospy velma_common rcprg_ros_utils roslib)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
catkin_install_python(PROGRAMS src/proj_1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS src/proj_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
