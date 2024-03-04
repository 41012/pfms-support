

PFMS Support - Setup
=========================

**PREREQUISITE:** you have installed ROS2 and created a workspace as per instructions on canvas in week 00 

On Azure we have installed pfms support packages. For your own devices refer [INSTALLATION](INSTALLATION.md)

If you get stuck in install there is a [Frequently Asked Questions - FAQ](./FAQ.md)

## Running Simulator

You can launch the simulator for the audi and husky. You will need to launch the simulator if your running any code that sends commands or receives data from the simulator. 

```
ros2 launch gazebo_tf audi_husky.launch.py
```
<img src="./images/rviz_audi_husky.png" alt="rviz_audi_husky" style="zoom:50%;" />

The terminal where you have executed this command is active, keep it running while you use the simulator (your testing your code). To terminate the simulator you have to execute CTRL+C in the terminal window.

Version Check
-------------------------

To check versions of your installed files if there is an update provided

```
dpkg -l | grep pipes
```

### Current versions

| package | version  X.Y.Z |
| ------- | -------------- |
| pipes   | 3.0.1          |

## Upgrades

**To update any of the libraries make sure `git pull` from `~/git/pfms-support`**

To update **pipes** library execute below (where you need to specify the correct package name, where you need to match the X.Y.Z at current version in table above your `ROSVERSION` (`humble`) and your system ( `amd64`). 

```bash
cd ~/git/pfms-support
sudo apt install ./package/pipes_X.Y.Z-ROSVERSION_SYSTEM.deb
sudo ldconfig
```

So it cold be `sudo apt install ./package/pipes_3.0.1-humble_amd64.deb`
If your working on any code that links to pipes (such as command_ugv) or your own assignment code. It is always good practice to rebuild it. So from build directory.

```
rm CMakeCache.txt
cmake ..
make
```

To update **all other packages** recompile your catkin work-space

```bash
cd ~/ros2_ws
colcon build --symlink-install
```
