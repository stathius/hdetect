cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()


# MESSAGES
rosbuild_genmsg() 

# EXECUTABLE NODES
#rosbuild_add_executable(annotateData src/annotateData.cpp)
#rosbuild_link_boost(annotateData signals)
rosbuild_add_executable(visualizeRT src/visualizeRT.cpp)
rosbuild_add_executable(headlessRT src/headlessRT.cpp)
rosbuild_link_boost(visualizeRT signals)
#rosbuild_add_executable(trainLaser src/trainLaser.cpp)
#rosbuild_add_executable(detectTest src/detectTest.cpp)
#rosbuild_add_executable(Calib_Matlab2ROS src/other/Calib_Matlab2ROS.cpp)

# LIBRARIES
rosbuild_add_library(laserLib src/lib/laserLib.cpp)
#rosbuild_add_library(annotator src/lib/annotator.cpp)
rosbuild_add_library(visualizer src/lib/visualizer.cpp)
#rosbuild_add_library(bagReader src/lib/bagReader.cpp)
rosbuild_add_library(lgeometry src/lib/lgeometry.cpp)
rosbuild_add_library(lfeatures src/lib/lfeatures.cpp)
rosbuild_add_library(lfeatures src/lib/lfeatures.cpp)
rosbuild_add_library(lengine src/lib/lengine.cpp)
rosbuild_add_library(projectTools src/lib/projectTools.cpp)
rosbuild_add_library(detector src/lib/detector.cpp)

# GSL IS NEEDED FOR PEOPLE2D LIBRARY TO COMPILE
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GLS_LIBRARY_DIRS})

# LINK THE LIBRARIES
target_link_libraries(lgeometry ${GSL_LIBRARIES})
target_link_libraries(lfeatures ${GSL_LIBRARIES})
target_link_libraries(lengine lgeometry lfeatures)
target_link_libraries(laserLib lengine)
target_link_libraries(detector laserLib projectTools)
target_link_libraries(visualizer detector)
#target_link_libraries(annotator visualizer)
#target_link_libraries(annotateData bagReader annotator)
target_link_libraries(visualizeRT visualizer lgeometry)
target_link_libraries(headlessRT detector lgeometry)
#target_link_libraries(detectTest visualizer)
