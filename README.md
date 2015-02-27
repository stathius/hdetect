A human detection and tracking package for ROS
=======

Uses a combination of laser range finder and a computer vision module for the pedestrian detection. The vision module works with OpenCV's detector which uses Histogram of Oriented Gradients and Support Vector Machines.

For pairing observations and tracking it uses a combination of Kalman filters and Mahalanobis distance.

The code was written for the [ROTOS](http://robcib.etsii.upm.es/index.php/en/projects-28/rotos) project in the RobCib research group of the Polytechnic University of Madrid.

## Publications

If you use this code please reference the following work:

[Fotiadis, E.P.; Garzón, M.; Barrientos, A.	Human Detection from a Mobile Robot Using Fusion of Laser and Vision Information. Sensors 2013, 13, 11603-11635.](http://www.mdpi.com/1424-8220/13/9/11603)

Bibtext entry:

```
@Article{s130911603,
AUTHOR = {Fotiadis, Efstathios P. and Garzón, Mario and Barrientos, Antonio},
TITLE = {Human Detection from a Mobile Robot Using Fusion of Laser and Vision Information},
JOURNAL = {Sensors},
VOLUME = {13},
YEAR = {2013},
NUMBER = {9},
PAGES = {11603--11635},
URL = {http://www.mdpi.com/1424-8220/13/9/11603},
PubMedID = {24008280},
ISSN = {1424-8220},
DOI = {10.3390/s130911603}
}
```

## Videos

[Human Detection with fusion of laser and camera](http://youtu.be/W84ERQ0LYjM)

[Autonomous detection tracking and following](http://youtu.be/W2lz9WmkjB0)

## Requirements
 1. ROS (groovy)
 2. libgsl
 3. all the libraries that are needed to compile the project
 
## How to compile

The code compiles only using *rosbuild*. The current version doesn't work with *catkin*.

* cd (hdetect folder)
* rosmake

## Executable files

* headlessRT - human detection without visualization and tracking
* visualizeRT - human detection with visualization and without tracking
* recognizeRT - human detection with tracking and without visualization
* showRT - human detection with visualization and tracking
* annotateData - annotate the human for training and save the result to csv file
* trainLaser – train the annotation with given csv file and save the result to boost.xml

## Launch files. 

It is suggested to run the launch files than to run the bin files

* headlessRT.launch - headlessRT on robot
* headlessBag.launch - headlessRT with rosbag. The bag name can be changed inside the launch
* visualizeRT.launch - visualizeRT on robot.
* visualizeBag.launch - visualizeRT with rosbag. The bag name can be changed inside the launch
* recognizeRT.launch - recognizeRT on robot.
* recognizeBag.launch - recognizeRT with rosbag. The bag name can be changed inside the launch
* showRT.launch - showRT on robot.
* showBag.launch - showRT with rosbag. The bag name can be changed inside the launch
* annotateBAG.launch - annotateData with rosbag. The bag name can be changed inside the launch.
* trainLaser.launch - trainLaser with csv file. The file name can be changed inside the launch.


##  How to see the tracking result with a bag
 1) “rosrun rviz rviz”. The config of rviz is at rviz/hdetect.rviz
If you can't see “No Image” on the left or grid on the right, redo the first step
 2) “roslaunch hdetect recognizeBag.launch”
 3) “rosbag play directory_of_bag”

## Brief explanation of the code

###lengine
segment the laser points into clusters, call the function to compute the 17 features of the laser points

###lfeature
compute the 17 features of the laser points

###lgeometry
compute the geometry used by the computation of the features

###laserLib
load the raw laser point, call the function to compute the clusters and the features

###projectTools
standard function for projection, used in everywhere

###Header
contains the enumeration of HUMAN, the static topic name, curTimeStamp and preTimeStamp


###Human
Structure for storing the value of the human detection and tracking


###Observation
Structure for storing the value casted from detection

###ObjectTracking
Using Kalman filter to track the object, including predict and update

###Detector
callback function of headlessRT, main function for detection, run the detection of laser and of image, then merge them together

###Visualizer
callback function of visualizeRT, run the detector first, then plot them on the window

###Recognizer
callback function of recognizeRT, run the detector first, then do the tracking of the human, and stand it to rviz

###Annotator
callback function of annotateData, main function of the annotation

###bagReader
read the bag for the annotation 
 
## Acknowledgements

The laser processing module uses code swritten by L. Spinello.  The tracking module is based on the work of Gonzalo Rodriguez-Canosa
