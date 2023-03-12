

FAQ
=========================

[TOC]

## [multi.launch] is neither a launch file in package [gazebo_tf] nor is [gazebo_tf] a launch file name

The error indicates ROS is unable to find the package that contains this file, which is tied to the simulation.

There are three possible reasons for this.

1) Your symbolic link is not correct

```bash
cd ~/catkin_ws/src
ls -la 
```
You should see <span style="color:teal">pfms_ros</span> in teal. If it is in red then your symbolic links is not correct. Not to despair we can simply re-link it, deleting a symbolic link is like removing a shortcut. You do need to find the location of your pfms_ros file which is in the pfms_support repository. For me this is ~/git/pfms_support/pfms_ros. If you accidentally checked out pfms_support elsewhere you can move it (using `mv` command), look up the syntax on the internet.

```bash
cd ~/catkin_ws/src
rm pfms_ros
ln -s ~/git/pfms_support/pfms_ros 
```
2) You have not compiled the packages

You need to execute `catkin_make` within the `catkin_ws` folder as per instructions. When catkin_make is running it will take some time to compile a lot of libraries and executable(s) and will give you update on progress `[ XX%]` where XX is a number from 0-100. You can also build individual packages so can try

```bash
cd ~/catkin_ws
catkin_make --pkg gazebo_tf
```

This will only compile gazebo_tf. If this produces only few lines an error check if you have actually customised Linux for PfMS, in particular section on [install ROS and all other required packages](https://canvas.uts.edu.au/courses/26214/pages/customising-linux-install-for-pfms?wrap=1) and read next section below about ~/.bashrc

3. You have not sourced your workspace

Check your ~/.bashrc file to see if your workspace had been configured correctly

```bash
tail ~/.bashrc
```

The last few lines should look something like

```bash
source /opt/ros/noetic/setup.bash
source /home/student/catkin_ws/devel/setup.bash
```

If your missing the `/opt/ros/` section execute `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`

If your missing the `/home/....` section (this is specific to your computer and your username then execute) `echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc`

Just sanity check again

```bash
tail ~/.bashrc
```

If it all looks good then execute `source ~/.bashrc`. This should not report errors (about unknown locations). if it does you need to remove those line that source the files. You can do so by using `nano` which is a editor

```bash
nano ~/.bashrc
```

Edit the file (used arrows/backspace to remove lines and then save CTRL+X and Y (follow prompts), execute `source ~/.bashrc` to check if the changes are OK.

