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
#### <u>Raising And Lowering The Forklift Fork</u>

The fork is mounted to the forklift body through a prismatic joint and is controlled with `ros2_control`.

Install the ROS2 control libriaries
```
sudo apt install ros-jazzy-ros-control
sudo apt install ros-jazzy-ros2-controllers 
sudo apt install ros-jazzy-ros2-control-demo
sudo apt install ros-jazzy-gz-ros2-control
```

You can manually raise and lower the fork with `teleop_twist_keyboard`.  After starting the main code with `rx20_16.launch.sh`, run the teleop in another terminal

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
and you will see this in your terminal
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
The fork can be raised and lowered with the `t` and `b` keys. The fork motion can be stopped at any point by pressing the `g` or `k` key.

#### <u>Docking With Apriltags</u>

To understand the Nav2 docking server and how to use Apriltag fiducial markers for localization, the best thing to do is to look at the Automatic Addison tutorials on the subject.

https://automaticaddison.com/autonomous-docking-with-apriltags-using-nav2-ros-2-jazzy/

https://www.youtube.com/watch?v=tFE75bP-ASo

The implementation for the forklift borrowed heavily from the above tutorial.

To run the basic docking example, run the `RX20_16.launch.sh` as described above.  If everything started correctly, in RViz you should see the depot map, the costmap, and the camera image facing the pallet in the distance.  In Gazebo you should see the forklift on one side of the open area and a pallet on the other end.

Zoom into the pallet and you will see AprilTags on all four sides of the pallet.  The docking server uses these to help guide the forklift when it is close enough to see the tag with the camera.

To dock, in RViz look for the docking panel.  In that panel, enter 'dock0' in the Dock id field and then click the 'Dock robot' button.  The robot will advance quickly to the staging pose about 2 meters from the pallet.  It will detected the AprilTag and move slowly to align to the AprilTab pose and dock with the pallet.  If the alignment is correct at about 10 cm distance from the AprilTag motion should stop and the docking state should will say "Reached".

Navigation to the staging pose is not reliable and sometimes the forklift will not see the AprilTag and docking will abort.  Other times the approch if off angle the fork cannot insert into the pallet.  This means navigation needs refinement.

#### <u>LLM with ROSA</u>

ROSA is framework that interfaces Langchain with ROS2.  In this code we have added ROSA so that you can use natural language commands to direct the forklift.  The current implementation uses a local LLM to reduce API latency and to save costs on development.

To install and use the local LLM you first need to get and install Ollama from https://ollama.com.

The model we use is `qwen3.5:4b` which is small enough to fit on mid-range GPUs but still performs well in chat, tool invocation, reasoning, and image processing for our narrow scope.

To download the model just run `ollama run qwen3.5:4b` in a terminal. This will download the model and start a chat session so you can try out the model.  You only need to do this once to get the model.  You can close this terminal after the model has been installed.

If you want to run a remote LLM like Claude or ChatGPT, you will need to edit `forklift_llm.py`.  There you will see an example of how to pick the service and set up your authentication.

To use ROSA with the forklift, first start the forklift code with the `RX20_16.launch.sh` shell script.  Then in a second terminal run in your ROS2 workspace `python ./install/wb_nav2_bringup/share/wb_nav2_bringup/scripts/forklift_llm_control.py`.  This will start a chat session with the forklift.

Commands that are known to work are

* Move forward
* Move backwards
* Turn right
* Turn left
* Stop
* Navigate to (x/y coordinates on the map)
* Dock to (dock name)

## To Do

* Add fork control to the gamepad teleop. (not a priority)
* Improve path planning to dock with a pallet so that the forklift approaches the pallet at an angle normal to the edge.
* Add friction to the pallet so that it doesn't move with a glancing blow to the pallet.  Increase the pallet lift capacity so it can pick up a weighted pallet.
* Expand the LLM code to handle high level commands that require a sequence of actions.

## History / Current State

v0.1: RX20 16 forklift with differential drive.  World is a small warehouse.  LiDAR streams data and can be visualized in rviz2. Navigation and localization work.

v0.2: RX20 16 forklift fork works and can pick up a pallet with manual control.

v0.3: Nav2 docking server and Apriltag detection are implemented. Docking a pallet with an attached apriltag works for easy conditions.

v0.4: Basic ROSA LLM is functional.
