# c3pzero
Monorepo for c3pzero mobile base.

# c3pzero
Monorepo for c3pzero mobile base.

## Getting started
On host
```
./build.sh
./run.sh
```
In container
```
sudo apt update
rosdep update
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build
source install/setup.bash
ros2 launch c3pzero_description display.launch.py
```
