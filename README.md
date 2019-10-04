# WeCook

> :warning: **Warning:** WeCook is under heavy development. These instructions are
> primarily for reference by the developers.

WeCook is a platform for robotic manipulation 
and human-robot collaboration research.

## Setup Instructions
**WeCook depends on [AIKIDO](https://github.com/personalrobotics/aikido), developed by the [Personal Robotics Lab](https://personalrobotics.cs.washington.edu/) at the University of Washington.** 

**WeCook is supported on the following systems:**
- Ubuntu 18.04 / ROS Melodic
- Ubuntu 16.04 / ROS Kinetic

### [Setup Instructions](docs/instructions.md)

## [Demo](#demo)
### Broccoli Beef
<p align="center">
<img src="https://github.com/icaros-usc/wecook/blob/master/docs/demo0.png" width="300">
</p>

**To run the demo, type in following commands in your terminal**

```bash
# Open one terminal
$ roscore
# Open another terminal
$ rviz
# Open another terminal
$ rosrun wecook wecook
# Open another terminal
$ rosrun wecook demo1.py
```



