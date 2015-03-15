cmake_minimum_required(VERSION 2.8.3)
project(audio_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS rospy audio_core)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
