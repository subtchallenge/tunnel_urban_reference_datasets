cmake_minimum_required(VERSION 2.8.3)
project(tf_hijacker)

set(PKG_DEPENDS
  tf
  roscpp
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  )
set(BUILD_DEPENDS
  ${PKG_DEPENDS}
  )
set(RUN_DEPENDS
  ${PKG_DEPENDS}
  )

#######################################
## Find dependencies
#######################################

find_package(catkin REQUIRED ${BUILD_DEPENDS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  )
#generate_dynamic_reconfigure_options(
#  cfg/LaserRetimer.cfg
#  )
set (CMAKE_CXX_STANDARD 11)
#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  )
add_executable(tf_hijacker src/tf_hijacker.cpp)
target_link_libraries(tf_hijacker ${catkin_LIBRARIES})

install(TARGETS tf_hijacker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#  )
#
