cmake_minimum_required(VERSION 2.8.3)
project(recognize_speech)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package()

# TODO: is this needed?
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)