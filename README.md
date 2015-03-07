Human detection & tracking package for ROS
=======

Uses a combination of laser range finder and a computer vision module for the pedestrian detection. The vision module works with OpenCV's detector which uses Histogram of Oriented Gradients and Support Vector Machines.

For pairing observations and tracking it uses a combination of Kalman filters and Mahalanobis distance.

The code was written for the [ROTOS](http://robcib.etsii.upm.es/) project in the RobCib research group of the Polytechnic University of Madrid.

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

Human Detection with fusion of laser and camera

[![Human Detection with fusion of laser and camera](http://img.youtube.com/vi/W84ERQ0LYjM/0.jpg)](http://www.youtube.com/watch?v=W84ERQ0LYjM)

Autonomous detection tracking and following

[![Autonomous detection tracking and following](http://img.youtube.com/vi/gqlUAyLwUE4/0.jpg)](http://www.youtube.com/watch?v=gqlUAyLwUE4)


## Requirements
 1. ROS (hydro)
 2. libgsl
 3. all the libraries that are needed to compile the project
 
## How to compile

The code compiles only using *rosbuild*. The current version doesn't work with *catkin*.

* cd (hdetect folder)
* rosmake

## Demo the code
 1. Download [this rosbag](https://www.dropbox.com/s/szi5szgs12amv99/moving7.bag?dl=0)
 2. Change the *recognizeBag.launch* launchfile to point towards the rosbag
 3. Run ```roslaunch hdetect recognizeBag.launch```
 4. Rviz is going to launch. Enable the image (camera)
 5. Wait till everything is launched and hit space to playback

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

## Brief explanation of the code

####lengine
segment the laser points into clusters, call the function to compute the 17 features of the laser points

####lfeature
compute the 17 features of the laser points

####lgeometry
compute the geometry used by the computation of the features

####laserLib
load the raw laser point, call the function to compute the clusters and the features

####projectTools
standard function for projection, used in everywhere

####Header
contains the enumeration of HUMAN, the static topic name, curTimeStamp and preTimeStamp


####Human
Structure for storing the value of the human detection and tracking

####Observation
Structure for storing the value casted from detection

####ObjectTracking
Using Kalman filter to track the object, including predict and update

####Detector
callback function of headlessRT, main function for detection, run the detection of laser and of image, then merge them together

####Visualizer
callback function of visualizeRT, run the detector first, then plot them on the window

####Recognizer
callback function of recognizeRT, run the detector first, then do the tracking of the human, and stand it to rviz

####Annotator
callback function of annotateData, main function of the annotation

####bagReader
read the bag for the annotation 
 
## Acknowledgements

The laser processing module uses code swritten by L. Spinello.  The tracking module is based on the work of Gonzalo Rodriguez-Canosa
