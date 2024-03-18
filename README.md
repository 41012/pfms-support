

PFMS Support - Setup
=========================

**PREREQUISITE:** 

- On Azure ROS2 and an workspace have been setup. 
- On your own device install ROS2 and created a workspace as per instructions on [canvas](https://canvas.uts.edu.au/courses/30581/pages/customising-linux-install-for-pfms?wrap=1).

Installation:

- On Azure we have installed pfms support packages. 
- For your own devices refer [INSTALLATION](INSTALLATION.md)

Always check if you need to update pipes (information below on how to check version and undertake an update). This does not occur frequently, a Teams Announcement is made to indicate a new version is released. If you get stuck in install or behaviour or running is odd there is a [Frequently Asked Questions - FAQ](./FAQ.md)

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
| pipes   | 3.0.2          |

## Upgrades

**To update any of the libraries make sure `git pull` from `~/git/pfms-support`**

To update **pipes** library execute below (where you need to specify the correct package name, where you need to match the X.Y.Z at current version in table above your `ROSVERSION` (`humble`) and your system ( `amd64`). 

```bash
cd ~/git/pfms-support
sudo apt install ./packages/pipes_3.0.2-humble_amd64.deb
sudo ldconfig
```

Just ignore apt related `W: ... _apt ...` warning lines. They're non-fatal, and for the most part you can't fix this, and you'll get the same results with or without the warning.

To update **all other packages** recompile your ros2 work-space

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Finally, any code that links to pipes (such as command_ugv) or your own assignment code needs to be rebuilt. Either vscode rebuild function or from build directory of your code you can execute:

```
rm CMakeCache.txt
cmake ..
make
```

