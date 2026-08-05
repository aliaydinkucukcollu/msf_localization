# Multi Sensor Fusion Localization

## Build & Launch

```bash
source /opt/ros/humble/setup.bash

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# optional: for vscode intellisense
ln -s build/compile_commands.json compile_commands.json

source install/setup.bash

ros2 launch msf_localization_ros2 msf_localization_ros2.launch.py
```

## Convert Bag ROS1 to ROS2

```bash

pip install rosbags

~/.local/bin/rosbags-convert ros1.bag --dst ros2_bag

```
