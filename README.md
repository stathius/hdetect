hdetect
=======

A human detection package for ROS written in C++. Uses a combination of laser range finder and a computer vision module.
The vision module works with OpenCV's detector which uses Histogram of Oriented Gradients and Support Vector Machines. The laser processing uses code from a library written by L. Spinello. 

Project website: http://robcib.etsii.upm.es/index.php/en/projects-28/rotos

Youtube video: http://youtu.be/W84ERQ0LYjM

v0.4

Detection is done only by the HoG detector. The laser is only used for localization and restricting the search space.
The module for extracting and annotating the laser clusters are almost finished and will be included in a future version.
