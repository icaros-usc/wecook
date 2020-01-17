# Setup Instructions
These instructions are tested on UBuntu 18.04.3 version of linux with 'melodic' version of ros.
If you faced any issues during following these instructions, consider updating this document.
(Talk to one of owners for help on how to update the document if you don't know).

## First you need catkin command line tools.
For more information about it, refer to https://catkin-tools.readthedocs.io/
**Run the below command in a terminal**
```bash
sudo apt-get install python-catkin-tools
```

## You need all things required for Aikido
Add a few custom PPAs necessary for Aikido
```
$ sudo add-apt-repository ppa:dartsim/ppa
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update
```

Install build dependencies for Dart/wecook
```
$ sudo apt install cmake build-essential libboost-filesystem-dev libmicrohttpd-dev libompl-dev libtinyxml2-dev libyaml-cpp-dev libccd-dev libfcl-dev liboctomap-dev libnlopt-dev liburdfdom-dev libcgal-dev
```

Build and install Dart from source
```
$ mkdir ~/ros_dependencies && cd ~/ros_dependencies
$ git clone https://github.com/dartsim/dart.git
$ cd dart
$ git checkout tags/v6.7.3
$ mkdir build && cd build
$ cmake ..
$ make -j4
$ sudo make install
```

Extend your shared library path to include libdart
```
$ echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/local/lib"' >> ~/.bashrc
$ source ~/.bashrc
```

Install some miscellaneous dependencies
```
$ sudo apt install ros-melodic-controller-interface ros-melodic-transmission-interface ros-melodic-realtime-tools ros-melodic-controller-manager ros-melodic-control-toolbox ros-melodic-octomap-ros ros-melodic-srdfdom python-catkin-tools python-pybind11
```

### Then there are additional dependencies required for wecook
Build and install Z3 from source (Additional reference: https://github.com/Z3Prover/z3/blob/master/README-CMake.md)
```
cd ~/ros_dependencies
git clone https://github.com/Z3Prover/z3.git
cd z3
git checkout tags/z3-4.8.7
mkdir build && cd build
cmake -G "Unix Makefiles" ../
make -j4
sudo make install
```

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
$ export ROS_ROOT_PATH=/opt/ros/melodic
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
