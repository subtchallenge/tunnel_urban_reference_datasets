cmake_minimum_required(VERSION 2.8.3)
project(packbot_visualization)

set(BUILD_DEPENDS
)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPENDS})

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS ${BUILD_DEPENDS}
#  DEPENDS system_lib
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

