#!/usr/bin/env python3
"""
Mission 7: Path Obstacle Evaluation - P3 to D1.

Places a single obstacle at various positions along the P3->D1 path
and measures success rate at each position.

Usage:
    python3 mission_7.py [--rounds N]

Examples:
    python3 mission_7.py --rounds 3
"""
import sys
import os
import time
import argparse
import csv
import subprocess
import math
from datetime import datetime

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from pallet_mission import (
    init, go_to_destination, dock, raise_fork, lower_fork,
    backup, go_home, shutdown, check_pallet_at_destination,
    is_robot_out_of_bounds, check_pallet_lifted
)

PALLET_ORIGINS = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.05, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.05, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.05, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.05, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.05, 'yaw': 0.0},
}

# Obstacle positions along P3(-14,0) -> D1(-5,-5.5) path
# Evenly spaced from near P3 to near D1
OBSTACLE_POSITIONS = [
    {'name': 'none',  'x': None,  'y': None},
    {'name': 'pos1',  'x': -13.0, 'y': -0.5},
    {'name': 'pos2',  'x': -11.5, 'y': -1.5},
    {'name': 'pos3',  'x': -10.0, 'y': -2.5},
    {'name': 'pos4',  'x': -8.5,  'y': -3.5},
    {'name': 'pos5',  'x': -7.0,  'y': -4.0},
    {'name': 'pos6',  'x': -5.5,  'y': -4.5},
]

WORLD_NAME = 'mission_depot_v1'
OBSTACLE_NAME = 'obstacle_1'
PALLET = 'P3'
DESTINATION = 'D1'


def spawn_obstacle(x, y, z=0.5):
    print(f"Spawning obstacle at ({x}, {y})...")
    req = (
        'sdf: \"<sdf version=\'1.6\'><model name=\'obstacle_1\'>'
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
    time.sleep(1.0)
    print("Obstacle spawned!")


def remove_obstacle():
    print("Removing obstacle...")
    subprocess.run([
        'gz', 'service',
        '-s', f'/world/{WORLD_NAME}/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "{OBSTACLE_NAME}" type: 2'
    ], capture_output=True)
    time.sleep(1.0)
    print("Obstacle removed!")


def reset_all_pallets():
    for pallet_name, origin in PALLET_ORIGINS.items():
        model_name = f'pallet_{pallet_name[1]}'
        yaw = origin['yaw']
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        subprocess.run([
            'gz', 'service',
            '-s', f'/world/{WORLD_NAME}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', f'name: "{model_name}", position: {{x: {origin["x"]}, y: {origin["y"]}, z: {origin["z"]}}}, orientation: {{x: 0, y: 0, z: {qz:.4f}, w: {qw:.4f}}}'
        ], capture_output=True)
        time.sleep(0.5)


def reset_pallet():
    origin = PALLET_ORIGINS[PALLET]
    model_name = f'pallet_{PALLET[1]}'
    yaw = origin['yaw']
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    subprocess.run([
        'gz', 'service',
        '-s', f'/world/{WORLD_NAME}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "{model_name}", position: {{x: {origin["x"]}, y: {origin["y"]}, z: {origin["z"]}}}, orientation: {{x: 0, y: 0, z: {qz:.4f}, w: {qw:.4f}}}'
    ], capture_output=True)
    time.sleep(1.0)


def run_mission():
    start_time = time.time()
    try:
        if not dock(PALLET):
            print("Docking failed, returning home...")
            lower_fork()
            go_home()
            reset_all_pallets()
            return False, time.time() - start_time
        raise_fork()
        if not check_pallet_lifted(PALLET):
            print("Pallet not lifted correctly, returning home...")
            lower_fork()
            go_home()
            reset_all_pallets()
            return False, time.time() - start_time
        if not go_to_destination(DESTINATION):
            print("Navigation failed, returning home...")
            lower_fork()
            reset_all_pallets()
            go_home()
            return False, time.time() - start_time
        lower_fork()
        backup()
        go_home()
        duration = time.time() - start_time
        success = check_pallet_at_destination(PALLET, DESTINATION)
        return success, duration
    except Exception as e:
        print(f"Mission failed with exception: {e}")
        lower_fork()
        go_home()
        reset_all_pallets()
        return False, time.time() - start_time


def print_report(results):
    print("\n" + "="*55)
    print("=== Forklift Mission 7 - Path Obstacle Evaluation ===")
    print(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"Route: {PALLET} → {DESTINATION}")
    print("="*55)

    total = len(results)
    successes = sum(1 for r in results if r['success'])
    print(f"\nTotal attempts: {total} | Successes: {successes} | Overall: {successes/total*100:.1f}%")

    print(f"\n--- By Obstacle Position ---")
    for pos in OBSTACLE_POSITIONS:
        pr = [r for r in results if r['position'] == pos['name']]
        if not pr:
            continue
        ps = sum(1 for r in pr if r['success'])
        avg = sum(r['duration_sec'] for r in pr) / len(pr)
        loc = f"({pos['x']}, {pos['y']})" if pos['x'] is not None else "none"
        print(f"{pos['name']:<6} {loc:<16}: {ps}/{len(pr)} ({ps/len(pr)*100:>5.1f}%) avg time: {avg:.1f}s")
    print("="*55)


def save_csv(results):
    filename = f"mission_7_results_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
    filepath = os.path.join(os.path.expanduser('~'), filename)
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['timestamp', 'position', 'obstacle_x', 'obstacle_y', 'success', 'duration_sec'])
        writer.writeheader()
        writer.writerows(results)
    print(f"\nRaw data saved to: {filepath}")
    return filepath


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=3)
    args = parser.parse_args()

    total = len(OBSTACLE_POSITIONS) * args.rounds
    print(f"=== Mission 7: Path Obstacle Evaluation ===")
    print(f"Route: {PALLET} → {DESTINATION}")
    print(f"Obstacle positions: {len(OBSTACLE_POSITIONS)}")
    print(f"Rounds per position: {args.rounds}")
    print(f"Total attempts: {total}")

    init()
    results = []
    attempt = 0

    for pos in OBSTACLE_POSITIONS:
        print(f"\n{'='*55}")
        if pos['x'] is not None:
            print(f"Obstacle position: {pos['name']} ({pos['x']}, {pos['y']})")
            spawn_obstacle(pos['x'], pos['y'])
        else:
            print(f"Obstacle position: none")

        try:
            for round_num in range(1, args.rounds + 1):
                attempt += 1
                print(f"\n[{attempt}/{total}] {PALLET} → {DESTINATION} (position: {pos['name']}, round: {round_num}/{args.rounds})")
                success, duration = run_mission()
                results.append({
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'position': pos['name'],
                    'obstacle_x': pos['x'],
                    'obstacle_y': pos['y'],
                    'success': success,
                    'duration_sec': round(duration, 1)
                })
                print(f"Result: {'SUCCESS' if success else 'FAILED'} ({duration:.1f}s)")
                reset_all_pallets()
                time.sleep(3.0)
                if is_robot_out_of_bounds():
                    print("Robot is out of map bounds! Aborting all missions.")
                    raise SystemExit(1)
        finally:
            if pos['x'] is not None:
                remove_obstacle()

    print_report(results)
    save_csv(results)
    shutdown()


if __name__ == '__main__':
    main()
