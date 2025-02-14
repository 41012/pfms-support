

PFMS Support - Setup
=========================

```bash
more /etc/ros/rosdep/sources.list.d/10-local.list
rosdep update
rosdep resolve audibot
rosdep check --from-paths . --ignore-src --rosdistro="$(rosversion -d)"

bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble
nano /home/student/ros2_ws/src/pfms-ros/audibot/rosdep_list.yaml
```

