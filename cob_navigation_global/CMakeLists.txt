cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_find_ros_package(actionlib_msgs)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# rostest
#rosbuild_add_roslaunch_check(launch/2dnav_linear.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(launch/2dnav_ros_dwa.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)
#rosbuild_add_roslaunch_check(launch/2dnav_ros_tr.launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)

#rosbuild_add_rostest(test/1_cob3-1.xml)
#rosbuild_add_rostest(test/2_cob3-1.xml)
#rosbuild_add_rostest(test/3_cob3-1.xml)
#
#rosbuild_add_rostest(test/1_cob3-2.xml)
#rosbuild_add_rostest(test/2_cob3-2.xml)
#rosbuild_add_rostest(test/3_cob3-2.xml)
#
#rosbuild_add_rostest(test/1_cob3-3.xml)
rosbuild_add_rostest(test/2_cob3-3.xml)
rosbuild_add_rostest(test/3_cob3-3.xml)
#
#rosbuild_add_rostest(test/1_cob3-4.xml)
#rosbuild_add_rostest(test/2_cob3-4.xml)
#rosbuild_add_rostest(test/3_cob3-4.xml)
#
#rosbuild_add_rostest(test/1_cob3-5.xml)
#rosbuild_add_rostest(test/2_cob3-5.xml)
#rosbuild_add_rostest(test/3_cob3-5.xml)
#
#rosbuild_add_rostest(test/1_cob3-6.xml)
#rosbuild_add_rostest(test/2_cob3-6.xml)
#rosbuild_add_rostest(test/3_cob3-6.xml)
#
#rosbuild_add_rostest(test/1_raw3-1.xml)
#rosbuild_add_rostest(test/2_raw3-1.xml)
#rosbuild_add_rostest(test/3_raw3-1.xml)
#
#rosbuild_add_rostest(test/1_raw3-2.xml)
#rosbuild_add_rostest(test/2_raw3-2.xml)
#rosbuild_add_rostest(test/3_raw3-2.xml)
#
#rosbuild_add_rostest(test/1_raw3-3.xml)
#rosbuild_add_rostest(test/2_raw3-3.xml)
#rosbuild_add_rostest(test/3_raw3-3.xml)
#
#rosbuild_add_rostest(test/1_desire.xml)
#rosbuild_add_rostest(test/2_desire.xml)
#rosbuild_add_rostest(test/3_desire.xml)

rosbuild_add_roslaunch_check(launch ROBOT=cob3-3 ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(launch robot:=cob3-3 robot_env:=ipa-kitchen)

