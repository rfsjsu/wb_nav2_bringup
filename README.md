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
git clone -b RX20_16_Forklift https://github.com/rfsjsu/wb_nav2_bringup.git
git clone -b jazzy https://github.com/pradyum/dual_laser_merger.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

To run the RX20_16 forklift in Gazebo Sim and RViz:
```
export WS_PATH="$HOME/ros2_ws"
cd ~/ros2_ws
source install/setup.bash
bash ./src/wb_nav2_bringup/scripts/rx20_16.launch.sh
```

To run the Turtlebot 4 demo:
```
ros2 launch wb_nav2_bringup tb4_simulation_launch.py headless:=False
```

To run the Turtlebot 3 demo:
```
ros2 launch wb_nav2_bringup tb3_simulation_launch.py headless:=False
```
<br>

### Topic Specific Documentation
---
#### Raising And Lowering The Forklift Fork

The fork is mounted to the forklift body through a prismatic joint and is controlled with `ros2_control`.

Install the ROS2 control libriaries
```
sudo apt install ros-jazzy-ros-control
sudo apt install ros-jazzy-ros2-controllers 
sudo apt install ros-jazzy-ros2-control-demo
sudo apt install ros-jazzy-gz-ros2-control
```

When the code starts, most of the time the controllers are not activated because the `controller_manager` hasn't finished initializing.  Run the following in a terminal to ensure the necessary controllers are running after you have run `rx20_16.launch.sh` and Gazebo has fully initialized.

```
ros2 control switch_controllers --activate joint_state_broadcaster
ros2 control switch_controllers --activate velocity_control
```

You can confirm the controllers are running with this command

```
ros2 control list_controllers
```
and the output should show the `joint_state_publisher` and `velocity_controller` are active

```
position_control        forward_command_controller/ForwardCommandController  inactive
velocity_control        forward_command_controller/ForwardCommandController  active  
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster        active
```

You can manually raise and lower the fork with `teleop_twist_keyboard` but you need to run `twist_to_multiarray.py` to turn z-axis twist messages into Float64MultiArray messages and publish them to the `velocity_controller`.

With `twist_to_multiarray.py` running, run the teleop in another terminal

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
and you see this in your terminal
```
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```
The fork can be raised and lowered with the `t` and `b` keys. The fork can be stopped at any point by pressing the `g` or `k` key.


## To Do

* TBD

## History / Current State

v0.1: RX20 16 forklift with differential drive.  World is a small warehouse.  LiDAR streams data and can be visualized in rviz2. Navigation and localization work.
