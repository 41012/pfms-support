# Gazebo Harmonic Migration Notes

## Changes Made

This package has been migrated from **Gazebo Classic** to **Gazebo Harmonic**.

### Key Changes:

1. **Dependencies Updated:**
   - Replaced `gazebo_msgs` with `ros_gz_interfaces`
   - Added `ros_gz_bridge` and `ros_gz_sim` as dependencies

2. **Topic Changes:**
   - **Old (Gazebo Classic):** `/demo/link_states_demo` (gazebo_msgs::msg::LinkStates)
   - **New (Gazebo Harmonic):** `/model/husky/pose` (geometry_msgs::msg::PoseStamped)

3. **Interface Changes:**
   - LinkStates message no longer exists in Gazebo Harmonic
   - Using PoseStamped for pose information
   - Velocity information should come from `/model/husky/odometry` topic

## Setting up ros_gz_bridge

To bridge Gazebo Harmonic topics to ROS 2, you need to configure `ros_gz_bridge`. Create a bridge configuration file or use command line:

### Option 1: Command Line Bridge
```bash
ros2 run ros_gz_bridge parameter_bridge /model/husky/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose
```

### Option 2: Launch File Configuration
Add to your launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/husky/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            '/model/husky/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    
    return LaunchDescription([bridge])
```

### Option 3: YAML Configuration File
Create `config/ros_gz_bridge.yaml`:
```yaml
- topic_name: "/model/husky/pose"
  ros_type_name: "geometry_msgs/msg/PoseStamped"
  gz_type_name: "gz.msgs.Pose"
  
- topic_name: "/model/husky/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
```

Then launch with:
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=config/ros_gz_bridge.yaml
```

## Gazebo Harmonic Services for Entity Control

In Gazebo Harmonic, to move entities programmatically:

### Set Entity Pose Service:
```bash
ros2 service call /world/default/set_pose gz_msgs/msg/Pose "{name: 'husky', position: {x: 1.0, y: 2.0, z: 0.5}}"
```

### Available Gazebo Services:
- `/world/<world_name>/set_pose` - Teleport entity to a pose
- `/world/<world_name>/create` - Spawn new entities
- `/world/<world_name>/remove` - Remove entities
- `/world/<world_name>/control` - Control simulation (play/pause)

## Running the Node

After setting up the bridge:
```bash
# Terminal 1: Start Gazebo Harmonic
ros2 launch <your_gazebo_launch>

# Terminal 2: Start the bridge (if not in launch file)
ros2 run ros_gz_bridge parameter_bridge /model/husky/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose

# Terminal 3: Start gazebo_connect node
ros2 run pfms gazebo_connect
```

## Notes:

1. The model name in topics (`/model/husky/pose`) must match your model name in the SDF file
2. Velocity information is not available in the PoseStamped message - consider subscribing to `/model/husky/odometry` if you need velocity
3. For full odometry data including velocity, modify the subscriber to use `nav_msgs/msg/Odometry` instead
