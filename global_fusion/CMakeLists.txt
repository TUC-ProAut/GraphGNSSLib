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

# ceres solver
find_package(Ceres REQUIRED)

# geographic lib for coordinate frame conversions
find_package(GeographicLib REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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

add_executable(testRTKLIBNode 
	src/testRTKLIBNode.cpp
	${src_folder_cpp}
        RTKLIB/src/ppp.c)
target_link_libraries(testRTKLIBNode ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(testRTKLIBNode PUBLIC cxx_std_14)


add_executable(gnss_preprocessor_node 
	src/gnss_preprocessor/gnss_preprocessor.cpp
	${src_folder_cpp}
        RTKLIB/src/ppp.c
        )
target_link_libraries(gnss_preprocessor_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
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
target_link_libraries(rosbag_generator_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(rosbag_generator_node PUBLIC cxx_std_14)

add_executable(psr_spp_node src/gnss_estimator/psr_spp.cpp)
target_link_libraries(psr_spp_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar ppp preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(psr_spp_node PUBLIC cxx_std_14)


add_executable(psr_spp_dyna_auto_node src/gnss_estimator/psr_spp_dyna_auto.cpp)
target_link_libraries(psr_spp_dyna_auto_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar ppp preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(psr_spp_dyna_auto_node PUBLIC cxx_std_14)


add_executable(psr_doppler_fusion_node 
	src/gnss_estimator/psr_doppler_fusion.cpp
        RTKLIB/src/lambda.cpp
	RTKLIB/src/ppp.c)
target_link_libraries(psr_doppler_fusion_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						)
target_compile_features(psr_doppler_fusion_node PUBLIC cxx_std_14)

add_executable(psr_doppler_car_rtk_node 
	src/gnss_estimator/psr_doppler_car_rtk.cpp
	RTKLIB/src/lambda.cpp
	RTKLIB/src/ppp.c)
target_link_libraries(psr_doppler_car_rtk_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(psr_doppler_car_rtk_node PUBLIC cxx_std_14)

add_executable(psr_doppler_car_rtk_dyna_node 
	src/gnss_estimator/psr_doppler_car_rtk_dyna.cpp
	RTKLIB/src/lambda.cpp
	RTKLIB/src/ppp.c)
target_link_libraries(psr_doppler_car_rtk_dyna_node ${catkin_LIBRARIES} Ceres::ceres Eigen3::Eigen ${GeographicLib_LIBRARIES}
						convkml convrnx datum download ephemeris geoid ionex 
						options ppp_ar preceph qzslex rcvraw rinex
						rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtksvr sbas solution
						stream streamsvr tle
						) 
target_compile_features(psr_doppler_car_rtk_dyna_node PUBLIC cxx_std_14)

add_executable(evaluate_gps_solution_node src/evo/evaluate_gps_solution.cpp)
target_link_libraries(evaluate_gps_solution_node 
                        ${catkin_LIBRARIES} 
                        Ceres::ceres
                        Eigen3::Eigen 
                        ${GeographicLib_LIBRARIES}
                        ) 
target_compile_features(evaluate_gps_solution_node PUBLIC cxx_std_14)

# message dependencies
add_dependencies(evaluate_gps_solution_node ${catkin_EXPORTED_TARGETS})

