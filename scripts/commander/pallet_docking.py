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
# pallet_positions = {
#     'pallet_A': [-10.0, 2.0, 1.57],
# }

# pallet id
pallet_ids = [
    'dock1',
    'dock2',
    'dock3',
    'dock4',
    'dock5'
]

# Shipping destination for picked products
shipping_destinations = [
    [-4.0, -4.0, 0.0],
    [-4.0, -2.0, 0.0],
    [-4.0, -0.0, 0.0],
    [-4.0, 2.0, 0.0],
    [-4.0, 4.0, 0.0],
]

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

def task_wait(navigator):
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print('.',end='',flush=True)
    print()

def fork_up():
    time.sleep(2)
    send_linear_z_vel(+1.0)
    time.sleep(3)
    send_linear_z_vel(0.0)
    time.sleep(2)
    return

def fork_down():
    time.sleep(2)
    send_linear_z_vel(-1.0)
    time.sleep(3)
    send_linear_z_vel(0.0)
    time.sleep(2)
    return

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
    # request_item_location = 'pallet_A'
    # request_destination = 'frieght_bay_1'
    ####################

    navigator = BasicNavigator()

    time.sleep(10)
    navigator.waitUntilNav2Active()

    fork_down()

    for id in [4,3,2,1,0]:

        # Spinning around seems to help AMCL get a more accurate pose estimation.
        navigator.spin(spin_dist=1.0, time_allowance=30)
        task_wait(navigator)

        navigator.dockRobotByID(pallet_ids[id])
        task_wait(navigator)
        print('Docking complete')

        fork_up()

        # Backup a bit before turning to new goal
        navigator.backup(backup_dist=0.5, backup_speed=0.5, time_allowance=20)
        task_wait(navigator)

        # Sping 180 degrees before turning to new goal
        navigator.spin(time_allowance=30)
        task_wait(navigator)

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
        shipping_destination.pose.position.x = shipping_destinations[id][0]
        shipping_destination.pose.position.y = shipping_destinations[id][1]
        shipping_destination.pose.orientation.z = 0.0
        shipping_destination.pose.orientation.w = 1.0
        navigator.goToPose(shipping_destination)
        task_wait(navigator)

        fork_down()

        # Backup a bit before turning to new goal
        navigator.backup(backup_dist=0.75, backup_speed=0.5, time_allowance=20)
        task_wait(navigator)

        # Add dropping action here again
        # If we stacked plates, the fork will be too high for the next pallet.
        #
        fork_down()

    exit(0)


if __name__ == '__main__':
    main()