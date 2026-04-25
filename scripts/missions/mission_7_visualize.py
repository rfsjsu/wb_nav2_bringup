#!/usr/bin/env python3
"""
Spawn all Mission 7 obstacle positions at once for visualization.
Run this to see all obstacle positions on the P3->D1 path in Gazebo.

Usage:
    python3 mission_7_visualize.py
    python3 mission_7_visualize.py --remove
"""
import subprocess
import time
import argparse

WORLD_NAME = 'mission_depot_v1'

OBSTACLE_POSITIONS = [
    {'name': 'pos1', 'x': -13.0, 'y': -0.5},
    {'name': 'pos2', 'x': -11.5, 'y': -1.5},
    {'name': 'pos3', 'x': -10.0, 'y': -2.5},
    {'name': 'pos4', 'x':  -8.5, 'y': -3.5},
    {'name': 'pos5', 'x':  -7.0, 'y': -4.0},
    {'name': 'pos6', 'x':  -5.5, 'y': -4.5},
]


def spawn_obstacle(name, x, y, z=0.5):
    print(f"Spawning {name} at ({x}, {y})...")
    req = (
        f'sdf: \"<sdf version=\'1.6\'><model name=\'{name}\'>'
        '<static>true</static><link name=\'link\'>'
        '<collision name=\'col\'><geometry><box><size>0.5 0.5 1.0</size></box></geometry></collision>'
        '<visual name=\'vis\'><geometry><box><size>0.5 0.5 1.0</size></box></geometry>'
        '<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual>'
        '</link></model></sdf>\", '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
    )
    subprocess.run([
        'gz', 'service',
        '-s', f'/world/{WORLD_NAME}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ], capture_output=True)


def remove_obstacle(name):
    print(f"Removing {name}...")
    subprocess.run([
        'gz', 'service',
        '-s', f'/world/{WORLD_NAME}/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "{name}" type: 2'
    ], capture_output=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--remove', action='store_true', help='Remove all obstacles')
    args = parser.parse_args()

    if args.remove:
        print("Removing all obstacle positions...")
        for pos in OBSTACLE_POSITIONS:
            remove_obstacle(pos['name'])
            time.sleep(0.5)
        print("All obstacles removed!")
    else:
        print("Spawning all Mission 7 obstacle positions...")
        for pos in OBSTACLE_POSITIONS:
            spawn_obstacle(pos['name'], pos['x'], pos['y'])
            time.sleep(0.5)
        print("All obstacles spawned!")
        print("\nObstacle positions:")
        for pos in OBSTACLE_POSITIONS:
            print(f"  {pos['name']}: ({pos['x']}, {pos['y']})")
        print("\nRun with --remove to remove all obstacles.")


if __name__ == '__main__':
    main()
