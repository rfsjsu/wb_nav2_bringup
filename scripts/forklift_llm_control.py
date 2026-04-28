from langchain_anthropic import ChatAnthropic
from langchain_ollama import ChatOllama
from langchain.agents import tool
from rosa import ROSA
from rosa.prompts import RobotSystemPrompts, system_prompts
import os
import pathlib
import time
import subprocess
from typing import Tuple
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose, DockRobot
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from forklift_help import get_help
from forklift_llm import get_llm
from forklift_prompts import get_prompts
from pallet_mission import (
    init, go_to_destination, dock, raise_fork, lower_fork,
    backup, go_home, shutdown, check_pallet_at_destination, PALLETS, DESTINATIONS
)
from std_msgs.msg import Bool, String, Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator

import base64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

bridge = CvBridge()
latest_image = None # Global to store latest frame

node = None
vel_publisher = None
vel_subscriber = None
explore_publisher = None
navigate_to_pose_action_client = None
dock_action_client = None
fork_publisher = None
navigator = None

def spin_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

def image_callback(msg):
    global latest_image
    latest_image = msg
    
def execute_ros_command(command: str) -> Tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)


# Helper function to get the maps directory
def _get_maps_dir() -> str:
    """Gets the absolute path to the 'maps' directory in the rosa_summit package, creates it if it doesn't exist."""

    maps_dir = "/home/ros/rap/Gruppe2/maps"
    pathlib.Path(maps_dir).mkdir(parents=True, exist_ok=True)
    return maps_dir

@tool
def pallet_mission(pallet: str, destination: str) -> str:
    """
    THIS IS THE ONLY TOOL TO USE for pallet pickup and delivery missions.
    Do NOT use dock_to_pallet or navigate tools separately for this.
    Available pallets: P1, P2, P3, P4, P5
    Available destinations: D1, D2

    :param pallet: Pallet name e.g. 'P1', 'P2'
    :param destination: Destination name e.g. 'D1', 'D2'
    """
    global node, vel_publisher, fork_publisher, navigator

    pallet_upper = pallet.upper()
    destination_upper = destination.upper()

    if pallet_upper not in PALLETS:
        return f"Invalid pallet... Choose from: {', '.join(PALLETS.keys())}"
    if destination_upper not in DESTINATIONS:
        return f"Invalid destination... Choose from: {', '.join(DESTINATIONS.keys())}"
    init(
        existing_node=node,
        existing_vel_publisher=vel_publisher,
        existing_fork_publisher=fork_publisher,
        existing_navigator=navigator
    )
    lower_fork()
    go_home()
    dock(pallet_upper)
    raise_fork()
    backup()
    go_to_destination(destination_upper)
    lower_fork()
    backup()
    go_home()
    return f"Mission complete... Delivred {pallet_upper} to destination {destination_upper}..."

@tool
def describe_scene() -> str:
    """
    Captures a single image from the camera and describes what is visable.
    Call this tool once per request. Do not call it multiple times.
    """
    
    global latest_image
    if latest_image is None:
        return "No camera image available"
    
    # Convert ROS image to OpenCV, then base64
    cv_image = bridge.imgmsg_to_cv2(latest_image, "bgr8")
    _, buffer = cv2.imencode(".jpeg", cv_image)
    image_64 = base64.b64encode(buffer).decode("utf-8")

    from langchain_ollama import ChatOllama
    from langchain_core.messages import HumanMessage

    vission_llm = ChatOllama(model="llama3.2-vision")
    message = HumanMessage(
        content=[
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_64}"}},
            {"type": "text", "text": "Describe the scene and focus on common warehouse objects and safety concerns."},
            ]
    )
    response = vission_llm.invoke([message])
    print(response)
    return response.content

@tool
def get_velocity() -> str:
    """
    Gets the linear and angular velocity of the robot.

    """
    print("DEBUG: Calling get_velocity()")
    return ""

@tool
def send_linear_x_vel(velocity: float) -> str:
    """
    Sets the forward or backward velocity of the robot.
    A positive velocity will move the forklift forward.
    A negative velocity will move the forklift backwards.

    :param velocity: the velocity at which the robot should move
    """
    print("DEBUG: Calling send_linear_x_vel() tool with x vel:",velocity)
    global vel_publisher
    twist = Twist()
    twist.linear.x = velocity
    vel_publisher.publish(twist)
    return "Linear x velocity set to %s" % velocity

@tool
def send_linear_z_vel(velocity: float) -> str:
    """
    Sets the linear z velocity of the robot to raise or lower the fork.
    A positive velocity will move the fork up.
    A negative velocity will move the fork down.

    :param velocity: the velocity at which the robot should move
    """
    print("DEBUG: Calling send_linear_z_vel() tool with z vel:",velocity)
    global vel_publisher
    twist = Twist()
    twist.linear.z = velocity
    vel_publisher.publish(twist)
    return "Linear z velocity set to %s" % velocity

@tool
def send_angular_z_vel(velocity: float) -> str:
    """
    Sets the angular velocity of the robot to make it turn.
    A positive angular velocity will turn the forklift right.
    A negative angular velocity will turn the forklift left.

    :param velocity: the velocity at which the robot should move
    """
    print("DEBUG: Calling send_angular_z_vel() tool with z ang vel:",velocity)
    global vel_publisher
    twist = Twist()
    twist.angular.z = velocity
    vel_publisher.publish(twist)
    return "Angular z velocity set to %s" % velocity

@tool
def stop() -> str:
    """
    Stops or halts the robot by setting its linear and angular velocities to zero.

    """
    print("DEBUG: Calling stop() tool")
    global vel_publisher
    twist = Twist()
    vel_publisher.publish(twist)
    return "Robot stopped"


@tool
def toggle_auto_exploration(resume_exploration: bool) -> str:
    """
    Starts or stops the autonomous exploration.

    :param resume_exploration: True to start/resume exploration, False to stop/pause exploration.
    """
    global explore_publisher
    msg = Bool()
    msg.data = resume_exploration
    explore_publisher.publish(msg)

    if resume_exploration:
        return "Autonomous exploration started/resumed."
    else:
        return "Autonomous exploration stopped/paused."


@tool
def navigate_to_pose(
    x: float, y: float, z_orientation: float, w_orientation: float
) -> str:
    """
    Moves the robot to an absolute position on the map.

    :param x: The x coordinate of the target position.
    :param y: The y coordinate of the target position.
    :param z_orientation: The z component of the target orientation (quaternion).
    :param w_orientation: The w component of the target orientation (quaternion).
    """
    print("DEBUG: Calling navigate_to_pose().")
    global navigate_to_pose_action_client, node

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}."

@tool
def dock_to_pallet(
    dock_id: str
) -> str:
    """
    Dock with a pallet.

    :param dock_id: The dock id used by the docking server to identify the pallet.
    """
    print("DEBUG: Calling dock_to pallet() with dock_id:",dock_id)
    global dock_action_client, node

    goal_msg = DockRobot.Goal()
    goal_msg.dock_pose.header.frame_id = "map"
    goal_msg.dock_pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.use_dock_id = True
    goal_msg.dock_id = dock_id
    goal_msg.max_staging_time = 120.0
    goal_msg.navigate_to_staging_pose = True

    dock_action_client.send_goal_async(goal_msg)
    return f"Dock goal sent to dock: {dock_id}."

@tool
def navigate_relative(
    x: float, y: float, z_orientation: float, w_orientation: float
) -> str:
    """
    Moves the robot relative to its current position using the robot's local coordinate frame.

    In the robot's coordinate system (base_link):
    - Positive X: Move forward
    - Negative X: Move backward
    - Positive Y: Move to the left
    - Negative Y: Move to the right

    :param x: The x coordinate of the target position relative to the robot (forward/backward).
    :param y: The y coordinate of the target position relative to the robot (left/right).
    :param z_orientation: The z component of the target orientation (quaternion) relative to the robot.
    :param w_orientation: The w component of the target orientation (quaternion) relative to the robot.
    """
    print("DEBUG: Calling navigate_relative().")
    global navigate_to_pose_action_client, node

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "base_link"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation.z = z_orientation
    goal_msg.pose.pose.orientation.w = w_orientation

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Relative navigation goal sent to x: {x}, y: {y}, orientation_z: {z_orientation}, orientation_w: {w_orientation}."


@tool
def save_map(map_name: str) -> str:
    """
    Saves the current map from the /summit/map topic to .yaml and .pgm files
    in the 'maps' directory of the 'rosa_summit' package.

    :param map_name: The name for the map (e.g., 'my_lab_map'). Do not include file extensions.
    """
    maps_dir = _get_maps_dir()
    if not os.path.isdir(
        maps_dir
    ):  # Should be created by _get_maps_dir, but double check
        return f"Error: Maps directory {maps_dir} could not be accessed or created."

    filepath_prefix = os.path.join(maps_dir, map_name)

    # Added --ros-args -r map:=/summit/map to specify the topic
    cmd = f"ros2 run nav2_map_server map_saver_cli -f '{filepath_prefix}' --ros-args -r map:=/summit/map"
    success, output = execute_ros_command(cmd)
    if success:
        if "Map saved to" in output:
            return f"Map successfully saved as {map_name} in {maps_dir}"
        else:
            return f"Map saving process initiated for {map_name} in {maps_dir}. Output: {output}"
    else:
        return f"Failed to save map {map_name} in {maps_dir}. Error: {output}"


@tool
def list_saved_maps() -> str:
    """
    Lists all saved maps in the 'maps' directory of the 'rosa_summit' package.
    Returns a list of map names (without .yaml extension).
    """
    maps_dir = _get_maps_dir()
    if not os.path.isdir(maps_dir):
        return "Maps directory not found or is not a directory."

    try:
        files = os.listdir(maps_dir)
        map_files = [
            f[:-5]
            for f in files
            if f.endswith(".yaml") and os.path.isfile(os.path.join(maps_dir, f))
        ]
        if not map_files:
            return "No saved maps found in the maps directory."
        return f"Available maps: {', '.join(map_files)}"
    except Exception as e:
        return f"Error listing maps: {e}"


@tool
def get_location_names() -> str:
    """
    Returns a list of available location names.
    """
    return f"Available locations: {', '.join(DESTINATIONS.keys())}"


@tool
def navigate_to_location_by_name(location_name: str) -> str:
    """
    Moves the robot to a predefined location by its name.

    :param location_name: The name of the location to navigate to (e.g., 'kitchen', 'gym').
    """
    global navigate_to_pose_action_client, node
    location_name_upper = location_name.upper()

    if location_name_upper not in DESTINATIONS:
        return f"Location '{location_name}' not found. Available: {', '.join(DESTINATIONS.keys())}"

    dest = DESTINATIONS[location_name_upper]
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = dest['x']
    goal_msg.pose.pose.position.y = dest['y']
    goal_msg.pose.pose.orientation.z = dest['oz']
    goal_msg.pose.pose.orientation.w = dest['ow']

    navigate_to_pose_action_client.send_goal_async(goal_msg)
    return f"Navigation goal sent to {location_name_upper}."

def main():
    global node, vel_publisher, explore_publisher, navigator
    global navigate_to_pose_action_client, dock_action_client, fork_publisher
    print("Hi from ROSA forklift.")

    # init rclpy
    rclpy.init()
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("rosa_forklift_node", parameter_overrides=[sim_time_param])
    spinner = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    spinner.start()    
    navigator = BasicNavigator(node_name='basic_navigator')
    navigator.waitUntilNav2Active()
    

    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    explore_publisher = node.create_publisher(Bool, "/explore/resume", 10)
    image_subscriber = node.create_subscription(Image, "/yolo/detections_image", image_callback, 1) # 1 keeps the last frame in queue
    fork_publisher = node.create_publisher(Float64MultiArray, "/velocity_control/commands", 10)
    navigate_to_pose_action_client = ActionClient( node, NavigateToPose, "/navigate_to_pose")
    dock_action_client = ActionClient(node, DockRobot, "/dock_robot")

    llm = get_llm()
    prompt = get_prompts()

    # Pass the LLM to ROSA with both tools available
    agent = ROSA(
        ros_version=2,
        llm=llm,
        tools=[
            send_linear_x_vel,
            send_linear_z_vel,
            send_angular_z_vel,
            stop,
            toggle_auto_exploration,
            navigate_to_pose,
            navigate_relative,
            save_map,
            list_saved_maps,
            get_location_names,
            navigate_to_location_by_name,
            dock_to_pallet,
            describe_scene,
            pallet_mission,
        ],
        prompts=prompt,
        verbose=True,
    )

    print("Type 'exit' or 'quit' to end the program")

    try:
        while True:
            msg = input("\nEnter your request: ")
            if msg.lower() in ["exit", "quit"]:
                break

            try:
                print("Request sent")
                res = agent.invoke(msg)
                if(isinstance(res, list)):
                    res = res[0]   # Need [0] if Claude, leave out if Ollama
                if isinstance(res, dict) and "text" in res:
                    print(res["text"])
                else:
                    print(res)
            except Exception as e:
                print(f"An error occurred: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")

    # agent.shutdown()
    print("Forklift LLM shutting down.")

if __name__ == "__main__":
    main()