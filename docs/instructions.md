# Setup Instructions
## Setting up the workspace
**Open a terminal and type in following commands**
```bash
# Store your favorite ros workspace path in a env variable ROS_WORKSPACE
# Feel free to replace ~/ros_ws with what you like
$ export ROS_WORKSPACE=~/ros_ws
$ mkdir $ROS_WORKSPACE && cd $ROS_WORKSPACE
$ wstool init src
$ catkin init
# Store your ros version in a env variable ROS_ROOT_PATH
# example: /opt/ros/kinetic or /opt/ros/melodic
$ export ROS_ROOT_PATH=/opt/ros/kinetic
$ catkin config --extend $ROS_ROOT_PATH
```
## Downloading the WeCook stack from github
**In the same termial, type in following commands**
```bash
$ cd ROS_WORKSPACE/src
$ git clone https://github.com/icaros-usc/wecook.git
$ wstool merge wecook/wecook.rosinstall
$ wstool up
```
## Building the workspace
**In the same terminal, type in following commands**
```bash
$ cd $ROS_WORKSPACE
$ catkin build
```
## Run demo
**Now you are ready to run all [demos](../README.md#usage)!**
