## Installation

If not done already:

```bash
cd ~/git
git clone git@github.com:41012/pfms-support.git
```

Proceed to install the pipes library, which has been supplied to allow using the physics simulator, at present bypassing the ROS framework for students.

```bash
cd pfms-support
sudo apt install ./packages/pipes_3.0.1-humble_amd64.deb
sudo ldconfig
```

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
sudo apt install python3-colcon-common-extensions
sudo apt install ros-dev-tools
```

Also check that your `.bashrc` file sources the ros `setup.bash` file. You can do so by
`tail ~/.bashrc`

Where you should see `source /opt/ros/humble/setup.bash`  displayed on screen. If it is missing then execute 

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

