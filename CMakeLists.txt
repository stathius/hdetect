cmake_minimum_required(VERSION 2.8.3)
project(hdetect)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  rosbag
  tf
  image_transport
  roscpp
  std_msgs
  cv_bridge
  laser_geometry
  angles
  sensor_msgs
  filters
  laser_filters
  message_filters
  pluginlib
  geometry_msgs
  image_geometry
  nav_msgs
  camera_calibration_parsers
  
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals)
find_library(NEWMAT newmat /usr/lib)

### GSL IS NEEDED FOR PEOPLE2D LIBRARY TO COMPILE
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSL REQUIRED gsl)
#include_directories(${GSL_INCLUDE_DIRS})
#link_directories(${GLS_LIBRARY_DIRS})

#find_package(newmat REQUIRED COMPONENTS newmat)
#find_package(GSL REQUIRED gsl)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ClusterClass.msg
   ClusteredScan.msg
   HumansFeatClass.msg
   HumansFeat.msg
 )

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   SetPolygon.srv
#   SetSimpleGoal.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs sensor_msgs std_msgs#   move_base_msgs#   nav_msgs#   visualization_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_follower
#  CATKIN_DEPENDS actionlib amp_ros geometry_msgs move_base_msgs nav_msgs roscpp tf visualization_msgs yaml-cpp
#  DEPENDS system_lib
)





###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${NEWMAT_INCLUDE_DIRS})
include_directories(${GSL_INCLUDE_DIRS})

link_directories(${GLS_LIBRARY_DIRS})


## Declare a cpp library
# add_library(trajectory_follower
#   src/${PROJECT_NAME}/trajectory_follower.cpp
# )


add_library(Header src/lib/header.cpp)
add_library(Recognizer src/lib/recognizer.cpp)
add_library(ObjectTracking src/lib/object_tracking.cpp)
add_library(Observation src/lib/observation.cpp)
add_library(Human src/lib/human.cpp)
add_library(laserLib src/lib/laserLib.cpp)
add_library(visualizer src/lib/visualizer.cpp)
add_library(lgeometry src/lib/lgeometry.cpp)
add_library(lfeatures src/lib/lfeatures.cpp)
add_library(lengine src/lib/lengine.cpp)
add_library(projectTools src/lib/projectTools.cpp)
add_library(detector src/lib/detector.cpp)
add_library(annotator src/lib/annotator.cpp)
add_library(bagReader src/lib/bagReader.cpp)

## Declare a cpp executable  # EXECUTABLE NODES
add_executable(visualizeRT src/visualizeRT.cpp)
add_executable(recognizeRT src/recognizeRT.cpp)
add_executable(showRT src/showRT.cpp)
add_executable(annotateData src/annotateData.cpp)
add_executable(trainLaser src/trainLaser.cpp)
add_executable(headlessRT src/headlessRT.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(visualizeRT hdetectr_generate_messages_cpp)
add_dependencies(recognizeRT hdetectr_generate_messages_cpp)
add_dependencies(showRT hdetectr_generate_messages_cpp)
add_dependencies(annotateData hdetectr_generate_messages_cpp)
add_dependencies(trainLaser hdetectr_generate_messages_cpp)
add_dependencies(headlessRT hdetectr_generate_messages_cpp)

add_dependencies(visualizer detector)
add_dependencies(Recognizer detector)
add_dependencies(detector laserLib)
add_dependencies(detector projectTools)
add_dependencies(laserLib lengine)
add_dependencies(lengine lgeometry)
add_dependencies(lengine lfeatures) 
add_dependencies(lengine Header) 
add_dependencies(annotateData bagReader)
add_dependencies(annotateData annotator)
add_dependencies(annotator visualizer)

add_dependencies(visualizeRT visualizer)
add_dependencies(visualizeRT lgeometry)

add_dependencies(headlessRT detector)
add_dependencies(headlessRT lgeometry)

add_dependencies(recognizeRT Recognizer)
add_dependencies(recognizeRT Humam)
add_dependencies(recognizeRT ObjectTracking)
add_dependencies(recognizeRT Observation)


## Specify libraries to link a library or executable target against
# target_link_libraries(trajectory_follower_node
#   ${catkin_LIBRARIES}
# )

# LINK THE LIBRARIES

target_link_libraries(visualizeRT ${Boost_LIBRARIES})
target_link_libraries(headlessRT ${Boost_LIBRARIES})
target_link_libraries(recognizeRT ${Boost_LIBRARIES})
target_link_libraries(showRT ${Boost_LIBRARIES})
target_link_libraries(annotateData ${Boost_LIBRARIES})

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


target_link_libraries(Header ${catkin_LIBRARIES})
target_link_libraries(Recognizer ${catkin_LIBRARIES})
target_link_libraries(ObjectTracking ${catkin_LIBRARIES})
target_link_libraries(Observation ${catkin_LIBRARIES})
target_link_libraries(Human ${catkin_LIBRARIES})
target_link_libraries(laserLib ${catkin_LIBRARIES})
target_link_libraries(visualizer ${catkin_LIBRARIES})
target_link_libraries(lgeometry ${catkin_LIBRARIES}) 
target_link_libraries(lfeatures ${catkin_LIBRARIES})
target_link_libraries(lengine ${catkin_LIBRARIES})
target_link_libraries(projectTools ${catkin_LIBRARIES})
target_link_libraries(detector ${catkin_LIBRARIES})
target_link_libraries(annotator ${catkin_LIBRARIES})
target_link_libraries(bagReader ${catkin_LIBRARIES})

target_link_libraries(visualizeRT ${catkin_LIBRARIES})
target_link_libraries(headlessRT ${catkin_LIBRARIES})
target_link_libraries(recognizeRT ${catkin_LIBRARIES})
target_link_libraries(showRT ${catkin_LIBRARIES})
target_link_libraries(annotateData ${catkin_LIBRARIES})
target_link_libraries(trainLaser ${catkin_LIBRARIES})

# )


# LIBRARIES

#find_library(NEWMAT newmat /usr/lib)





#target_link_libraries(detectTest visualizer)
