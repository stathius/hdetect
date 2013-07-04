#!/bin/bash
source ~/ros_workspace/setup.bash
roscd
tar zcf - hdetect | ssh summit 'tar zxf -'
ssh summit 'rm -rf ~/ros_workspace/hdetect'
ssh summit 'mv -vf ~/hdetect ~/ros_workspace'
