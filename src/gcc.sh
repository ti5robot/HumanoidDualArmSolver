#!/bin/bash
# export CPLUS_INCLUDE_PATH=/mnt/hgfs/VM共享文件夹/mechanical_arm_7_axis_two_arms_two_can/include:$CPLUS_INCLUDE_PATH
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ti5robot/mechanical_arm_7_axis_double_can/include/can
g++  main.cpp  mathfunc.cpp Ti5BASIC.cpp  Ti5MOVE.cpp Ti5LOGIC.cpp tool.cpp Ti5CAN_Driver.cpp   -L../include/can  -lmylibscan -lcontrolcan   -o move_sov
