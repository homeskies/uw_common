cmake_minimum_required(VERSION 2.8.12)
project(uw_knowledge)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)