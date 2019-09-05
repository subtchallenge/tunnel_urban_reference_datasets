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
  include
  ${catkin_INCLUDE_DIRS}
  )
generate_dynamic_reconfigure_options(
  cfg/LaserRetimer.cfg
  )
set (CMAKE_CXX_STANDARD 11)
#######################################
## Declare Catkin package and installation
#######################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${RUN_DEPENDS}
  DEPENDS
  )
add_executable(tf_ortho_duplicate src/tf_ortho_duplicate.cpp)
target_link_libraries(tf_ortho_duplicate ${catkin_LIBRARIES})

add_executable(tf_hijacker src/tf_hijacker.cpp)
target_link_libraries(tf_hijacker ${catkin_LIBRARIES})

add_executable(tf_multiply src/tf_multiply.cpp)
target_link_libraries(tf_multiply ${catkin_LIBRARIES})

add_executable(laser_retimer src/laser_retimer.cpp)
target_link_libraries(laser_retimer ${catkin_LIBRARIES})
add_dependencies(laser_retimer ${PROJECT_NAME}_gencfg)

add_executable(laser_replicatrix src/laser_replicatrix.cpp)
target_link_libraries(laser_replicatrix ${catkin_LIBRARIES})

add_executable(tf_poser src/tf_poser.cpp)
target_link_libraries(tf_poser ${catkin_LIBRARIES})

install(TARGETS tf_hijacker tf_multiply laser_retimer laser_replicatrix tf_poser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )

