## Installation

If you have already installed pipes and your ros workspace, and can go back to [README](README.md) and check if you need an update.

If not then follow below:

```bash
cd ~/git
git clone git@github.com:41012/pfms-support.git
```

Proceed to install the pipes library, which has been supplied to allow using the physics simulator, at present bypassing the ROS framework for students.

```bash
cd pfms-support
sudo apt install ./packages/pipes_3.0.5-humble_amd64.deb
sudo ldconfig
```
If you get following warning you can ignore it `N: Download is performed unsandboxed as root as file`, it's just stating the package is installed from a local file.

If not done already, link the `pfms_ros` folder to your `ros2_ws/src`

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s ~/git/pfms-support/pfms-ros 
```

Now we can make the package.

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/.bashrc
```

**You should now have all the required software**.  and can go back to [README](README.md)

If you have an error such as

```bash
CMake Error at CMakeLists.txt:15 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.
```

Then try again installing the tools

```bash
sudo apt install ros-dev-tools
```

Also check that your `.bashrc` file sources the ros `setup.bash` file. You can do so by
`tail ~/.bashrc`

Where you should see `source /opt/ros/humble/setup.bash`  and something like `source /home/XXXXXX/ros2_ws/install/setup.bash` displayed on screen (wheer `XXXX` is your username and thus it depends on your device username. The `/opt/ros` are packages supplied by ros and part of system while those in  `ros2_ws` are those installed for our subject.

If `/opt/ros` it is missing in your `~/.bashrc` then 

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
If the `ros_ws`  is missing in your `~/.bashrc` then 

```bash
echo "source ${HOME}/ros2_ws/install/setup.bash" >> ~/.bashrc
```
Then execute below to update the system, this makes your system aware of software installed in the two locations (all packages and support files).
```bash
source ~/.bashrc
```

