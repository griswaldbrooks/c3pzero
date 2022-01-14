# URDF for c3pzero
In spite of the name, this is a differential drive robot.

```
sudo apt update
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build
source install/setup.bash
ros2 launch c3pzero_description display.launch.py

```
