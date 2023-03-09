

Assignment 1 - Setup
=========================

**PRE-REQUISITE:** you have installed ROS and created a workspace as per instructions on canvas in week 00. 

**ASSUMPTION**: you have cloned this repository under your ~/git folder.

UPDATED: 09 MARCH 19:41

```
cd ~/git
git clone git@github.com:41012/pfms-support.git
```

## Installation

Proceed to install the pipes library, which has been supplied to allow using the physics simulator, at present bypassing the ROS framework for students.

Please select the pipes package for the version of ROS you have

```bash
cd pfms-support
cd packages
sudo dpkg -i pipes-2.2.0-noetic-Linux.deb
```

Then link the `a1_ros` folder to your `catkin_ws/src`

```bash
cd ~/catkin_ws/src
ln -s ~/git/pfms-support/pfms-ros 
```
Now we can make the package.

```bash
cd ~/catkin_ws
catkin_make
```

You should now have all the required software. 

You can launch the simulator for the audi and drone

```
roslaunch gazebo_tf multi.launch
```
<img src="./images/rviz_multi.png" alt="rviz_multi" style="zoom:50%;" />

The terminal where you have executed this command is active, keep it running while you use the simulator (your testing your code). To terminate the simulator you have to execute CTRL+C in the terminal window.

You can run other commands in another terminal. There is also a gui for the quadcopter that you need to run from another terminal () (just for sanity checking), hit `Z` first to enable control.

```bash
rosrun sjtu_drone drone_keyboard
```

Terminate this GUI by clicking the CANCEL button.

Version Check
-------------------------

To check versions of your installed files if there is an update provided

```
rosversion audibot_gazebo
rosversion gazebo_tf
rosversion sjtu_drone
dpkg -l | grep pipes
```

## Current versions

| package        | version |
| -------------- | ------- |
| audibot_gazebo | 0.2.1   |
| gazebo_tf      | 0.1.2   |
| sjtu_drone     | 0.0.0   |
| pipes          | 2.2.0   |

### Upgrades

To update a library make sure `git pul` from `~/git/pfms-support`

To update **pipes** library execute below (where you need to specify the correct package name, ie `pipes-2.2.0-noetic.Linux.deb` instead of `pipes-latest-Linux.deb` in the below.

```bash
cd ~/git/pfms-support/packages
sudo dpkg -i pipes-latest-Linux.deb
```
To update all other libraries recompile your catkin workspace

```bash
cd ~/catkin_ws
catkin_make
```

