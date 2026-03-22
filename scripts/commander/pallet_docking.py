#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import asyncio
import time

node = None
vel_publisher = None

# pallet positions for picking
pallet_positions = {
    'pallet_A': [-10.0, 2.0, 1.57],
}

# pallet id
pallet_ids = [
    'dock3',
]

# Shipping destination for picked products
shipping_destinations = {
    'frieght_bay_1': [-4.0, 0.0, 0.0],
}

def send_linear_z_vel(velocity: float) -> str:
    """
    Sets the linear z velocity of the robot to raise or lower the fork.
    A positive velocity will move the fork up.
    A negative velocity will move the fork down.

    :param velocity: the velocity at which the robot should move
    """
    if(velocity > 0.0):
        print("Raising the fork")
    elif(velocity < 0.0):
        print("Lowering the fork")
    global vel_publisher
    twist = Twist()
    twist.linear.z = velocity
    vel_publisher.publish(twist)
    return "Linear z velocity set to %s" % velocity

"""
Basic item picking demo. In this demonstration, the expectation
is that there is a person at the item pallet to put the item on the robot
and at the pallet jack to remove it
(probably with some kind of button for 'got item, robot go do next task').
"""


def main():

    rclpy.init()

    # From ROSA code
    global node, vel_publisher
    sim_time_param = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
    node = rclpy.create_node("rosa_forklift_node", parameter_overrides=[sim_time_param])
    vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)

    # Recieved virtual request for picking item at Shelf A and bring to
    # worker at the pallet jack 7 for shipping. This request would
    # contain the pallet ID ('pallet_A') and shipping destination ('frieght_bay_3')
    ####################
    request_item_location = 'pallet_A'
    request_destination = 'frieght_bay_1'
    ####################

    navigator = BasicNavigator()

    time.sleep(10)
    navigator.waitUntilNav2Active()

    navigator.dockRobotByID(pallet_ids[0])

    # Do something during our route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Simply print information for workers on the robot's ETA for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.',end='')
    print()
    print('Docking complete')

    # Add lifting action here
    #
    time.sleep(2)
    send_linear_z_vel(1.0)
    time.sleep(4)
    send_linear_z_vel(0.0)
    time.sleep(2)

    # Backup a bit before turning to new goal
    print("Backing up")
    navigator.backup(backup_dist=0.5, backup_speed=0.5, time_allowance=20)
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.',end='')
    print()
    print("Back up complete")

    # Sping 180 degrees before turning to new goal
    print("Spin around")
    navigator.spin()
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.',end='')
    print()
    print("Spin complete")

    # Go to drop off point
    # To convert the yaw angle in radians to quaterion z and w
    # z = sin(yaw/2)
    # w = cos(yaw/2)
    #
    # yaw = 0 is pointing right, so z=0, w=1
    #
    shipping_destination = PoseStamped()
    shipping_destination.header.frame_id = 'map'
    shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    shipping_destination.pose.position.x = shipping_destinations[
        request_destination
    ][0]
    shipping_destination.pose.position.y = shipping_destinations[
        request_destination
    ][1]
    shipping_destination.pose.orientation.z = 0.0
    shipping_destination.pose.orientation.w = 1.0
    navigator.goToPose(shipping_destination)
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.',end='')
    print()
    print("At the drop off point")

    # Add lifting action here
    #
    time.sleep(2)
    send_linear_z_vel(-1.0)
    time.sleep(5)
    send_linear_z_vel(0.0)
    time.sleep(2)

    # Backup a bit before turning to new goal
    print("Backing up")
    navigator.backup(backup_dist=1.5, backup_speed=0.5, time_allowance=20)
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('.',end='')
    print()
    print("Back up complete")

    exit(0)


if __name__ == '__main__':
    main()