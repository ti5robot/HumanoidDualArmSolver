#!/bin/bash
# export CPLUS_INCLUDE_PATH=/mnt/hgfs/VM共享文件夹/mechanical_arm_7_axis_two_arms_two_can/include:$CPLUS_INCLUDE_PATH
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ti5robot/mechanical_arm_7_axis_double_can/include/can
g++  main.cpp     -L../include  -lmylibti5_2004 -lcontrolcan   -o move_sov
