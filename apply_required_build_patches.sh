#!/bin/sh
cd kimera_ws/src/dbow2_catkin
git apply ../../../patches/dbow2catkin_build_patch.diff
cd ../gflags_catkin
git apply ../../../patches/gflagscatkin_build_patch.diff
cd ../glog_catkin
git apply ../../../patches/glogcatkin_build_patch.diff
cd ../Kimera-VIO
git apply ../../../patches/kimeravio_build_patch.diff
cd ../opengv_catkin
git apply ../../../patches/opengvcatkin_build_patch.diff
cd ../../..
