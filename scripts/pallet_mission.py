#!/usr/bin/env python3
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import subprocess

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
    'P5': 'dock5',
}

# Staging positions per pallet (2m in front of dock face, facing the dock)
# dock yaw -> approach direction -> robot quaternion
# dock1: yaw=+90deg (+Y) -> approach from -Y -> robot faces +Y: oz=0.7071, ow=0.7071
# dock2,3,4: yaw=0deg (+X) -> approach from -X -> robot faces +X: oz=0.0, ow=1.0
# dock5: yaw=-90deg (-Y) -> approach from +Y -> robot faces -Y: oz=-0.7071, ow=0.7071
# DOCK_STAGING = {
#     'P1': {'x': -10.0, 'y': -5.0, 'oz': -0.7071, 'ow': 0.7071},
#     'P2': {'x': -12.0, 'y': -3.0, 'oz': 0.0,    'ow': 1.0},
#     'P3': {'x': -12.0, 'y':  0.0, 'oz': 0.0,    'ow': 1.0},
#     'P4': {'x': -12.0, 'y': 3.0, 'oz': 0.0,    'ow': 1.0},
#     'P5': {'x': -10.0, 'y':  8.0, 'oz': -0.7071, 'ow': 0.7071},
# }

# Drop-off destinations
DESTINATIONS = {
    'D2': {'x': -5.0, 'y':  5.5, 'oz': 0.7071,  'ow': 0.7071},
    'D1': {'x': -5.0, 'y': -5.5, 'oz': -0.7071, 'ow': 0.7071},
    'NONE': None,
}

def init(existing_node=None, existing_vel_publisher=None, existing_fork_publisher=None, existing_navigator=None):
    global node, vel_publisher, fork_publisher, navigator

    if existing_node is not None:
        # Use existing ROS node and publisher instead of creating a new one
        node = existing_node
        vel_publisher = existing_vel_publisher
        fork_publisher = existing_fork_publisher
        navigator = existing_navigator
    
    else:
        rclpy.init()
        sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        node = rclpy.create_node("pallet_mission_node", parameter_overrides=[sim_time_param])
        vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
        fork_publisher = node.create_publisher(Float64MultiArray, "/velocity_control/commands", 10)
        time.sleep(10)
        navigator = BasicNavigator()
        print("Waiting for Nav2...")
        navigator.waitUntilNav2Active()
        print("Nav2 ready!")

def set_gz_view():
    '''
    Set Gazebo to an overhead view of the depot.  How to do this is described here:
    https://automaticaddison.com/how-to-change-the-default-gazebo-camera-view/
    '''
    subprocess.run([
        'gz', 'service',
        '-s', '/gui/move_to/pose',
        '--reqtype', 'gz.msgs.GUICamera',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', "pose: {position: {x: -7.385, y: 0.0847, z: 9.5} orientation: {x: -0.4791, y: 0.4814, z: 0.5178, w: 0.5202}}",
    ], capture_output=True)
    return

def wait_for_task(timeout=60.0, debug=False):
    i = 0
    start = time.time()
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if(debug):
            if feedback and i % 5 == 0:
                print('.', end='', flush=True)
            if time.time() - start > timeout:
                print()
                print("Timeout! Cancelling task...")
                navigator.cancelTask()
                break
    if(debug):
        print()
    return

def wait_until_in_zone(cx, cy, radius=1.5, timeout=120.0, debug=False):
    """목적지 zone 안에 들어오면 태스크 취소하고 종료"""
    import math
    i = 0
    start = time.time()
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            if(debug): print('.', end='', flush=True)
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
    time.sleep(1.0)
    msg.data = [1.0]
    fork_publisher.publish(msg)
    time.sleep(duration)
    msg.data = [0.0]
    fork_publisher.publish(msg)
    time.sleep(1.0)
    print("Fork raised!")

def lower_fork(duration=5.0):
    print("Lowering fork...")
    msg = Float64MultiArray()
    time.sleep(1.0)
    msg.data = [-1.0]
    fork_publisher.publish(msg)
    time.sleep(duration)
    msg.data = [0.0]
    fork_publisher.publish(msg)
    time.sleep(1.0)
    print("Fork lowered!")

def dock(pallet_name, timeout=120.0, debug=False):
    dock_id = PALLETS[pallet_name]
    if(debug): print(f"Docking to {pallet_name} ({dock_id})...")
    navigator.dockRobotByID(dock_id)
    time.sleep(1.0)  # wait for Nav2 to register the docking task
    wait_for_task(timeout=timeout)
    if(debug): print(f"Docking complete!")
    return navigator.getResult()

def backup(distance=1.5, speed=0.5, debug=False):
    if(debug): print(f"Backing up {distance}m...")
    navigator.backup(backup_dist=distance, backup_speed=speed, time_allowance=20)
    wait_for_task()
    if(debug): print("Backup complete!")
    return

def spin(debug=False):
    if(debug): print("Spinning around")
    navigator.spin()
    wait_for_task()
    if(debug): print("Spin complete")
    return

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

def go_home(debug=False):
    if(debug): print("Going home...")
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
    if(debug): print("Home!")

def go_to_destination(dest_name):
    dest = DESTINATIONS[dest_name]
    go_to(dest['x'], dest['y'], dest.get('oz', 0.0), dest.get('ow', 1.0))

# def spin():
#     print("Spinning...")
#     navigator.spin()
#     wait_for_task()
#     print("Spin complete!")

def shutdown():
    rclpy.shutdown()

def check_pallet_at_destination(pallet_name, dest_name, radius=2.0):
    """Check if pallet is within radius of destination using gz topic."""
    import subprocess, math, re
    dest = DESTINATIONS[dest_name]
    pallet_model = f'pallet_{pallet_name[1]}'
    result = subprocess.run(
        ['gz', 'topic', '-e', '-n', '1', '-t', '/world/mission_depot_v2/pose/info'],
        capture_output=True, text=True, timeout=5
    )
    pattern = rf'name: "{pallet_model}".*?position {{.*?x: ([-\d.e]+).*?y: ([-\d.e]+)'
    match = re.search(pattern, result.stdout, re.DOTALL)
    if not match:
        print(f"Could not find {pallet_model} position")
        return False
    px, py = float(match.group(1)), float(match.group(2))
    dist = math.sqrt((px - dest['x'])**2 + (py - dest['y'])**2)
    print(f"{pallet_model} at ({px:.2f}, {py:.2f}), distance to {dest_name}: {dist:.2f}m")
    return dist < radius

def pallet_movement_from_origin(pallet_name, starting_poses):
    """
    Return the distance between the pallet's current pose to the starting pose
    before docking.  This is intended to be used after docking and before lifting
    the pallet to quantify any movement of the pallent by a misaligned fork
    hitting the pallet.
    """
    import subprocess, math, re
    start = starting_poses[pallet_name]
    pallet_model = f'pallet_{pallet_name[1]}'
    result = subprocess.run(
        ['gz', 'topic', '-e', '-n', '1', '-t', '/world/mission_depot_v2/pose/info'],
        capture_output=True, text=True, timeout=5
    )
    pattern = rf'name: "{pallet_model}".*?position {{.*?x: ([-\d.e]+).*?y: ([-\d.e]+)'
    match = re.search(pattern, result.stdout, re.DOTALL)
    if not match:
        print(f"Could not find {pallet_model} position")
        return False
    px, py = float(match.group(1)), float(match.group(2))
    dist = math.sqrt((px - start['x'])**2 + (py - start['y'])**2)
    return dist


# ============== RSF CODE ==================
def send_linear_z_vel(velocity: float) -> str:
    """
    Sets the linear z velocity of the robot to raise or lower the fork.
    A positive velocity will move the fork up.
    A negative velocity will move the fork down.

    :param velocity: the velocity at which the robot should move
    """
    global vel_publisher
    twist = Twist()
    twist.linear.z = velocity
    vel_publisher.publish(twist)
    return "Linear z velocity set to %s" % velocity

def task_wait(navigator, progress=False):
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            if(progress):
                print('.',end='',flush=True)
    print()

def fork_up():
    time.sleep(1)
    for i in range(5):
        send_linear_z_vel(+1.0)
        time.sleep(1)
    send_linear_z_vel(0.0)
    time.sleep(1)
    return

def fork_down():
    time.sleep(1)
    for i in range(8):
        send_linear_z_vel(-1.0)
        time.sleep(1)
    send_linear_z_vel(0.0)
    time.sleep(1)
    return
