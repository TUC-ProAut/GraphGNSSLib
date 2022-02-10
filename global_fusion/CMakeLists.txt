cmake_minimum_required(VERSION 3.0)
project(global_fusion)

# We build as Release by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# path for custom find scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# path for locally installed dependencies
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/ThirdParty/install)

# basic catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nlosExclusion
  novatel_oem7_msgs
  message_filters
)

# Eigen
find_package(Eigen3 REQUIRED)

# glog & gflags
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
find_package(gflags REQUIRED)

# geographic lib for coordinate frame conversions
find_package(GeographicLib REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
)

catkin_package()

# add rtklib
FILE(GLOB src_folder_c "${PROJECT_SOURCE_DIR}/RTKLIB/src/*.c")
FILE(GLOB src_folder_cpp "${PROJECT_SOURCE_DIR}/RTKLIB/src/*.cpp")
#~ add_library(rtklib_core ${src_folder_c})

add_library(convkml RTKLIB/src/convkml.c)
add_library(convrnx RTKLIB/src/convrnx.c)
add_library(datum RTKLIB/src/datum.c)
add_library(download RTKLIB/src/download.c)
add_library(ephemeris RTKLIB/src/ephemeris.c)
add_library(geoid RTKLIB/src/geoid.c)
add_library(ionex RTKLIB/src/ionex.c)
add_library(options RTKLIB/src/options.c)
add_library(ppp_ar RTKLIB/src/ppp_ar.c)
add_library(ppp RTKLIB/src/ppp.c)
add_library(preceph RTKLIB/src/preceph.c)
add_library(qzslex RTKLIB/src/qzslex.c)
add_library(rcvraw RTKLIB/src/rcvraw.c)
add_library(rinex RTKLIB/src/rinex.c)
add_library(rtcm RTKLIB/src/rtcm.c)
add_library(rtcm2 RTKLIB/src/rtcm2.c)
add_library(rtcm3 RTKLIB/src/rtcm3.c)
add_library(rtcm3e RTKLIB/src/rtcm3e.c)
add_library(rtkcmn RTKLIB/src/rtkcmn.c)
add_library(rtksvr RTKLIB/src/rtksvr.c)
add_library(sbas RTKLIB/src/sbas.c)
add_library(solution RTKLIB/src/solution.c)
add_library(stream RTKLIB/src/stream.c)
add_library(streamsvr RTKLIB/src/streamsvr.c)
add_library(tle RTKLIB/src/tle.c)

add_executable(gnss_preprocessor_node 
	src/gnss_preprocessor/gnss_preprocessor.cpp
	${src_folder_cpp}
        RTKLIB/src/ppp.c
        )
target_link_libraries(gnss_preprocessor_node ${catkin_LIBRARIES} Eigen3::Eigen ${glog_LIBRARIES} ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(gnss_preprocessor_node PUBLIC cxx_std_14)

add_executable(rosbag_generator_node 
	src/gnss_preprocessor/rosbag_generator.cpp
	${src_folder_cpp}
	RTKLIB/src/ppp.c)
target_link_libraries(rosbag_generator_node ${catkin_LIBRARIES} Eigen3::Eigen ${glog_LIBRARIES} ${gflags_LIBRARIES}  ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(rosbag_generator_node PUBLIC cxx_std_14)
