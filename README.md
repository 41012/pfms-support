Assignment 1 - Setup
=========================

**PRE-REQUISITE:** you have installed ROS and created a workspace as per instructions on canvas in week 00. 

**ASSUMPTION**: you have cloned this repository under your ~/git folder.

```
git clone git@github.com:41012/pfms-support.git
```

## Installation

Proceed to install the pipes library, which has been supplied to allow using the physics simulator, at present bypassing the ROS framework for students.

Please select the pipes package for the version of ROS you have

```bash
cd packages
sudo dpkg -i pipes-2.1.0-noetic-Linux.deb
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

There is also a gui for the quadcopter (just for sanity checking), hit `Z` first to enable control.

```bash
rosrun sjtu_drone drone_keyboard
```

to terminate the simulator you have to execute CTRL+C in the terminal window.

Version Check and Updates
-------------------------

To check versions of your installed files if there is an update provided

```
rosversion audibot_gazebo
rosversion gazebo_tf
rosversion sjtu_drone
dpkg -l | grep pipes
```

If you need to update a library make sure `git pul`



Then, update the pipes library, and recompile your catkin workspace

```bash
cd ~/git/pfms-support/packages
sudo dpkg -i pipes-latest-Linux.deb
```
```bash
cd ~/catkin_ws
catkin_make
```

