# warehouse_bot
MSAI project: Autonomous forklift

This code originated from the Nav2 turtlebot 4 tutorial and a forklift model I found on github:

* https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation
* https://github.com/Marcus-techtile/pallet_detection/tree/main/src/forklift_simulator

## How To Build And Run The Code

### Prerequisites to run the code:
This code was developed with
* Linux Mint 22.3 (equivalent to Ubuntu 24.04)
* ROS2 Jazzy (including`ros-jazzy-ros-gz`)
* Python 3.12
* Nav2

### In addition, it depends on the following to compile:
* Colcon
* Cmake

To build:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rfsjsu/wb_nav2_bringup.git
git clone -b jazzy https://github.com/pradyum/dual_laser_merger.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

To run Gazebo Sim and RViz:
```
cd ~/ros2_ws
source install/setup.bash
bash ./src/wb_nav2_bringup/scripts/rx20_16.launch.sh
```

### Topic Specific Documentation

* TBD

## To Do

* TBD

## History / Current State

v0.1: RX20 16 forklift with differential drive.  World is a small warehouse.  LiDAR streams data and can be visualized in rviz2. Navigation and localization work.
