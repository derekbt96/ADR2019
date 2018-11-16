#!/bin/bash

# bebop_auto requirements
sudo apt-get install ros-kinetic-rviz
sudo apt-get install ros-kinetic-robot-state-publisher
sudo apt-get install python-tf
sudo apt-get install python-pygame
sudo apt-get install python-imaging-tk
sudo apt-get install ros-kinetic-tf
python -m pip install pygame
sudo python -m pip install https://pypi.python.org/packages/source/g/getch/getch-1.0-python2.tar.gz

# bebop_autonomy
sudo apt-get install build-essential python-rosdep python-catkin-tools

# run catkin_make in workspace
sudo ~/catkin_ws/catkin_make
# close all terminals
exit

