#!/bin/bash
source ~/ros_workspace/setup.bash
roscd hdetect
rsync -zv --exclude=.svn --exclude=CMakeCache.txt * summit:~/ros_workspace/hdetect/
rsync -zvr --exclude=.svn msg summit:~/ros_workspace/hdetect/
rsync -zvr --exclude=.svn src summit:~/ros_workspace/hdetect/
rsync -zvr --exclude=.svn include summit:~/ros_workspace/hdetect
rsync -zvr --exclude=.svn yaml summit:~/ros_workspace/hdetect/
rsync -zvr --exclude=.svn calibration\ parameters summit:~/ros_workspace/hdetect/
rsync -zvr --exclude=.svn launches summit:~/ros_workspace/hdetect/
rsync -zvr --exclude=.svn rviz summit:~/ros_workspace/hdetect/
