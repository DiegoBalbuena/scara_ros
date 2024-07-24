#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
<<<<<<< HEAD
: ${_CATKIN_SETUP_DIR:=/home/diego/scara_ros/install}
=======
: ${_CATKIN_SETUP_DIR:=/home/rabios/dev/scara_ros/install}
>>>>>>> a939d1951b49873009e624302dcc1bd2adec4208
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS
