#!/usr/bin/env python3
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

node = None
vel_publisher = None
fork_publisher = None
navigator = None

# Known locations on the map
LOCATIONS = {
    'home': {'x': -2.0, 'y': 0.0, 'z': 0.9999, 'w': 0.0016},
}

# Pallet dock IDs
PALLETS = {
    'P1': 'dock1',
    'P2': 'dock2',
    'P3': 'dock3',
    'P4': 'dock4',
}

# Staging positions per pallet (2m in front of dock face, facing the dock)
# dock yaw -> approach direction -> robot quaternion
# dock1: yaw=+90deg (+Y) -> approach from -Y -> robot faces +Y: oz=0.7071, ow=0.7071
# dock2,3,4: yaw=0deg (+X) -> approach from -X -> robot faces +X: oz=0.0, ow=1.0
# dock5: yaw=-90deg (-Y) -> approach from +Y -> robot faces -Y: oz=-0.7071, ow=0.7071
DOCK_STAGING = {
    'P1': {'x': -10.0, 'y': -5.0, 'oz': -0.7071, 'ow': 0.7071},
    'P2': {'x': -12.0, 'y': -3.0, 'oz': 0.0,    'ow': 1.0},
    'P3': {'x': -12.0, 'y':  0.0, 'oz': 0.0,    'ow': 1.0},
    'P4': {'x': -10.0, 'y':  8.0, 'oz': -0.7071, 'ow': 0.7071},
}

# Drop-off destinations
DESTINATIONS = {
    'D2': {'x': -5.0, 'y':  5.5},
    'D1': {'x': -5.0, 'y': -5.5},
}

def init():
    global node, vel_publisher, fork_publisher, navigator
    rclpy.init()
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("pallet_mission_node", parameter_overrides=[sim_time_param])
    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
    fork_publisher = node.create_publisher(Float64MultiArray, "/velocity_control/commands", 10)
    navigator = BasicNavigator()
    print("Waiting for Nav2...")
    time.sleep(10)
    navigator.waitUntilNav2Active()
    print("Nav2 ready!")

def wait_for_task(timeout=60.0):
    i = 0
    start = time.time()
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.', end='', flush=True)
        if time.time() - start > timeout:
            print()
            print("Timeout! Cancelling task...")
            navigator.cancelTask()
            break
    print()

def wait_until_in_zone(cx, cy, radius=1.5, timeout=120.0):
    """목적지 zone 안에 들어오면 태스크 취소하고 종료"""
    import math
    i = 0
    start = time.time()
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.', end='', flush=True)
            rx = feedback.current_pose.pose.position.x
            ry = feedback.current_pose.pose.position.y
            dist = math.sqrt((rx - cx)**2 + (ry - cy)**2)
            if dist < radius:
                navigator.cancelTask()
                print()
                print(f"Arrived in zone! (distance: {dist:.2f}m)")
                return
        if time.time() - start > timeout:
            print()
            print("Timeout! Cancelling task...")
            navigator.cancelTask()
            return
        time.sleep(0.1)
    print()

def raise_fork(duration=4.0):
    print("Raising fork...")
    msg = Float64MultiArray()
    msg.data = [1.0]
    fork_publisher.publish(msg)
    time.sleep(duration)
    msg.data = [0.0]
    fork_publisher.publish(msg)
    print("Fork raised!")

def lower_fork(duration=5.0):
    print("Lowering fork...")
    msg = Float64MultiArray()
    msg.data = [-1.0]
    fork_publisher.publish(msg)
    time.sleep(duration)
    msg.data = [0.0]
    fork_publisher.publish(msg)
    print("Fork lowered!")

def dock(pallet_name):
    dock_id = PALLETS[pallet_name]
    print(f"Docking to {pallet_name} ({dock_id})...")
    navigator.dockRobotByID(dock_id)
    time.sleep(1.0)  # wait for Nav2 to register the docking task
    wait_for_task(timeout=120.0)
    print(f"Docking complete!")

def backup(distance=1.5, speed=0.5):
    print(f"Backing up {distance}m...")
    navigator.backup(backup_dist=distance, backup_speed=speed, time_allowance=20)
    wait_for_task()
    print("Backup complete!")

def go_to(x, y, oz=0.0, ow=1.0):
    print(f"Navigating to ({x}, {y})...")
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = oz
    goal.pose.orientation.w = ow
    navigator.goToPose(goal)
    time.sleep(1.0)  # wait for Nav2 to register the new task
    wait_for_task()
    print(f"Arrived at ({x}, {y})!")

def go_home():
    print("Going home...")
    loc = LOCATIONS['home']
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = loc['x']
    goal.pose.position.y = loc['y']
    goal.pose.orientation.z = loc['z']
    goal.pose.orientation.w = loc['w']
    navigator.goToPose(goal)
    time.sleep(1.0)
    wait_for_task(timeout=120.0)
    print("Home!")

def go_to_destination(dest_name):
    dest = DESTINATIONS[dest_name]
    go_to(dest['x'], dest['y'])

def spin():
    print("Spinning...")
    navigator.spin()
    wait_for_task()
    print("Spin complete!")

def shutdown():
    rclpy.shutdown()
