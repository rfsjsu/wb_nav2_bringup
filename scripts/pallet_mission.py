#!/usr/bin/env python3
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

node = None
vel_publisher = None
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
    'P5': 'dock5',
}

# Drop-off destinations
DESTINATIONS = {
    'D1': {'x': -7.0, 'y':  0.0},
    'D2': {'x': -5.0, 'y':  5.5},
    'D3': {'x': -5.0, 'y': -5.5},
}

def init():
    global node, vel_publisher, navigator
    rclpy.init()
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("pallet_mission_node", parameter_overrides=[sim_time_param])
    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
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
    twist = Twist()
    twist.linear.z = 1.0
    vel_publisher.publish(twist)
    time.sleep(duration)
    twist.linear.z = 0.0
    vel_publisher.publish(twist)
    print("Fork raised!")

def lower_fork(duration=5.0):
    print("Lowering fork...")
    twist = Twist()
    twist.linear.z = -1.0
    vel_publisher.publish(twist)
    time.sleep(duration)
    twist.linear.z = 0.0
    vel_publisher.publish(twist)
    print("Fork lowered!")

def dock(pallet_name):
    dock_id = PALLETS[pallet_name]
    print(f"Docking to {pallet_name} ({dock_id})...")
    navigator.dockRobotByID(dock_id)
    wait_for_task()
    print(f"Docking complete!")

def backup(distance=0.5, speed=0.5):
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
    wait_until_in_zone(loc['x'], loc['y'], radius=1.5)
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
