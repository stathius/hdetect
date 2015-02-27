hdetect
=======

A human detection package for ROS written in C++. Uses a combination of laser range finder and a computer vision module.
The vision module works with OpenCV's detector which uses Histogram of Oriented Gradients and Support Vector Machines. The laser processing module uses code swritten by L. Spinello. 

Project website: http://robcib.etsii.upm.es/index.php/en/projects-28/rotos

Youtube video: http://youtu.be/W84ERQ0LYjM

## Usage:

This is the technical documentation for Human Detection (hdetect)

 1. Requirement
 1) ROS (groovy)
 2) libgsl
 3) all the libraries that are needed to compile the project
 2. Compile
 1) “cd (directory of hdetect)”
 2) “cmake .”
 3) “rosmake”
 4) If there are some libraries missing, use apt-get to download the missing ones and redo from 2.2
 3. IDE for coding
 1) qtcreator (I use this one because it is easy to deal with c++ and cmake )
download it from apt-get
type “qtcreator CMakeList” to open the project
Ctrl + b to compile
 2) vim, eclipse, etc
 4. Executable code and launch in hdetect
 1) There are 6 executable file in hdetect
headlessRT - human detection without visualization and tracking
visualizeRT - human detection with visualization and without tracking
recognizeRT - human detection with tracking and without visualization
showRT - human detection with visualization and tracking
annotateData - annotate the human for training and save the result to csv file
trainLaser – train the annotation with given csv file and save the result to boost.xml
 2) There are 10 executable file in hdetect. Its suggested to run the launch file than to run the bin file
headlessRT.launch - headlessRT on robot
headlessBag.launch - headlessRT with rosbag. The bag name can be changed inside the launch
visualizeRT.launch - visualizeRT on robot.
visualizeBag.launch - visualizeRT with rosbag. The bag name can be changed inside the launch
recognizeRT.launch - recognizeRT on robot.
recognizeBag.launch - recognizeRT with rosbag. The bag name can be changed inside the launch
showRT.launch - showRT on robot.
showBag.launch - showRT with rosbag. The bag name can be changed inside the launch
annotateBAG.launch - annotateData with rosbag. The bag name can be changed inside the launch.
trainLaser.launch - trainLaser with csv file. The file name can be changed inside the launch.
 5. How to see the tracking result with a bag
 1) “rosrun rviz rviz”. The config of rviz is at rviz/hdetect.rviz
If you can't see “No Image” on the left or grid on the right, redo the first step
 2) “roslaunch hdetect recognizeBag.launch”
 3) “rosbag play directory_of_bag”
 6. How to add a new code in CmakeList
 1) For executable file (for those that has int main())
rosbuild_add_executable(bin_name file_name_1 file_name_2 ...)
 2) For library file
rosbuild_add_library(variable_name file_name_1 file_name_2 …)
 7. Breve explanation for each code
 1) lengine
segment the laser points into clusters, call the function to compute the 17 features of the laser points
 2) lfeature
compute the 17 features of the laser points
 3) lgeometry
compute the geometry used by the computation of the features
 4) laserLib
load the raw laser point, call the function to compute the clusters and the features
 5) projectTools
standard function for projection, used in everywhere
 6) Header
contains the enumeration of HUMAN, the static topic name, curTimeStamp and preTimeStamp
 7) Human
Structure for storing the value of the human detection and tracking
 8) Observation
Structure for storing the value casted from detection
 9) ObjectTracking
Using Kalman filter to track the object, including predict and update
 10) detector
callback function of headlessRT, main function for detection, run the detection of laser and of image, then merge them together
 11) visualizer
callback function of visualizeRT, run the detector first, then plot them on the window
 12) recognizer
callback function of recognizeRT, run the detector first, then do the tracking of the human, and stand it to rviz
 13) annotator
callback function of annotateData, main function of the annotation
 14) bagReader
read the bag for the annotation
 8. Other
 1) If you have any problem, do not hesitate to ask me. My email is: b98902084@ntu.edu.tw 
