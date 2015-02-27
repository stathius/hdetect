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
#set(ROS_COMPILE_FLAG "-fno-stack-protector")

rosbuild_add_boost_directories()


# MESSAGES
rosbuild_genmsg() 

# EXECUTABLE NODES

rosbuild_add_executable(visualizeRT src/visualizeRT.cpp)
rosbuild_add_executable(headlessRT src/headlessRT.cpp)
rosbuild_add_executable(recognizeRT src/recognizeRT.cpp)
rosbuild_add_executable(showRT src/showRT.cpp)
rosbuild_link_boost(visualizeRT signals)
rosbuild_link_boost(headlessRT signals)
rosbuild_link_boost(recognizeRT signals)
rosbuild_link_boost(showRT signals)

rosbuild_add_executable(annotateData src/annotateData.cpp)
rosbuild_link_boost(annotateData signals)
rosbuild_add_executable(trainLaser src/trainLaser.cpp)
#rosbuild_add_executable(detectTest src/detectTest.cpp)
#rosbuild_add_executable(Calib_Matlab2ROS src/other/Calib_Matlab2ROS.cpp)

# LIBRARIES

find_library(NEWMAT newmat /usr/lib)

rosbuild_add_library(Header src/lib/header.cpp)

rosbuild_add_library(Recognizer src/lib/recognizer.cpp include/hdetect/lib/recognizer.hpp)
rosbuild_add_library(ObjectTracking src/lib/object_tracking.cpp include/hdetect/lib/object_tracking.hpp)
rosbuild_add_library(Observation src/lib/observation.cpp)
rosbuild_add_library(Human src/lib/human.cpp)

rosbuild_add_library(laserLib src/lib/laserLib.cpp include/hdetect/lib/laserLib.hpp)
rosbuild_add_library(visualizer src/lib/visualizer.cpp include/hdetect/lib/visualizer.hpp)
rosbuild_add_library(lgeometry src/lib/lgeometry.cpp include/hdetect/lib/lgeometry.hpp)
rosbuild_add_library(lfeatures src/lib/lfeatures.cpp include/hdetect/lib/lfeatures.hpp)
rosbuild_add_library(lengine src/lib/lengine.cpp include/hdetect/lib/lengine.hpp)
rosbuild_add_library(projectTools src/lib/projectTools.cpp include/hdetect/lib/projectTools.hpp)
rosbuild_add_library(detector src/lib/detector.cpp include/hdetect/lib/detector.hpp)

rosbuild_add_library(annotator src/lib/annotator.cpp include/hdetect/lib/annotator.hpp)
rosbuild_add_library(bagReader src/lib/bagReader.cpp include/hdetect/lib/bagReader.hpp include/hdetect/lib/bagSubscriber.hpp)


# GSL IS NEEDED FOR PEOPLE2D LIBRARY TO COMPILE
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GLS_LIBRARY_DIRS})

# LINK THE LIBRARIES
target_link_libraries(lgeometry ${GSL_LIBRARIES})
target_link_libraries(lfeatures ${GSL_LIBRARIES})
target_link_libraries(lengine lgeometry lfeatures Header)
target_link_libraries(laserLib lengine)
target_link_libraries(detector laserLib projectTools)
target_link_libraries(visualizer detector)
target_link_libraries(Recognizer detector)
target_link_libraries(visualizeRT visualizer lgeometry  ${NEWMAT})
target_link_libraries(headlessRT detector lgeometry ${NEWMAT})
target_link_libraries(recognizeRT Recognizer lgeometry ${NEWMAT} Recognizer ObjectTracking Human Observation)
target_link_libraries(showRT ${GSL_LIBRARIES})

target_link_libraries(annotator visualizer)
target_link_libraries(annotateData bagReader annotator)
#target_link_libraries(detectTest visualizer)
