#!/usr/bin/env python3
"""
Mission 6: Reliability Evaluation with Obstacle Avoidance.

Same as Mission 3 but with a static obstacle placed between the home zone
and the pallet area. Tests Nav2 obstacle avoidance capability.

Usage:
    python3 mission_6.py [--rounds N] [--pallets P1 P2 ...] [--destinations D1 D2 ...]

Examples:
    python3 mission_6.py --rounds 1
    python3 mission_6.py --rounds 15
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
    init, go_to, go_to_destination, dock, raise_fork, lower_fork,
    backup, go_home, shutdown, check_pallet_at_destination, PALLETS, DESTINATIONS,
    is_robot_out_of_bounds, check_pallet_lifted
)

PALLET_ORIGINS = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.05, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.05, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.05, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.05, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.05, 'yaw': 0.0},
}

WORLD_NAME = 'mission_depot_v1'
OBSTACLE_NAME = 'obstacle_1'
OBSTACLE_X = -8.0
OBSTACLE_Y = 0.0
OBSTACLE_Z = 0.5
OBSTACLE_SIZE = '0.5 0.5 1.0'


def spawn_obstacle():
    print(f"Spawning obstacle at ({OBSTACLE_X}, {OBSTACLE_Y})...")
    req = (
        'sdf: "<sdf version=\'1.6\'><model name=\'obstacle_1\'>'
        '<static>true</static><link name=\'link\'>'
        '<collision name=\'col\'><geometry><box><size>0.5 0.5 1.0</size></box></geometry></collision>'
        '<visual name=\'vis\'><geometry><box><size>0.5 0.5 1.0</size></box></geometry>'
        '<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual>'
        '</link></model></sdf>", '
        f'pose: {{position: {{x: {OBSTACLE_X}, y: {OBSTACLE_Y}, z: {OBSTACLE_Z}}}}}'
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
    for pallet_name in PALLET_ORIGINS:
        reset_pallet(pallet_name)


def reset_pallet(pallet_name):
    print(f"Resetting {pallet_name} to origin...")
    origin = PALLET_ORIGINS[pallet_name]
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
    time.sleep(1.0)
    print(f"{pallet_name} reset done!")


def run_mission(pallet, destination):
    start_time = time.time()
    try:
        if not dock(pallet):
            print("Docking failed, returning home...")
            lower_fork()
            reset_all_pallets()
            go_home()
            return False, time.time() - start_time
        raise_fork()
        if not check_pallet_lifted(pallet):
            print("Pallet not lifted correctly, returning home...")
            lower_fork()
            reset_all_pallets()
            go_home()
            return False, time.time() - start_time
        if not go_to_destination(destination):
            print("Navigation to destination failed, returning home...")
            lower_fork()
            reset_all_pallets()
            go_home()
            return False, time.time() - start_time
        lower_fork()
        reset_all_pallets()
        backup()
        go_home()
        duration = time.time() - start_time
        success = check_pallet_at_destination(pallet, destination)
        return success, duration
    except Exception as e:
        print(f"Mission failed with exception: {e}")
        lower_fork()
        reset_all_pallets()
        go_home()
        return False, time.time() - start_time


def print_report(results):
    print("\n" + "="*55)
    print("=== Forklift Mission 6 - Obstacle Avoidance Evaluation ===")
    print(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"Obstacle: {OBSTACLE_SIZE}m box at ({OBSTACLE_X}, {OBSTACLE_Y})")
    print("="*55)

    total = len(results)
    successes = sum(1 for r in results if r['success'])
    print(f"\n--- Overall ---")
    print(f"Total attempts: {total}")
    print(f"Total successes: {successes}")
    print(f"Overall success rate: {successes/total*100:.1f}%")

    print(f"\n--- By Pallet ---")
    for p in sorted(set(r['pallet'] for r in results)):
        pr = [r for r in results if r['pallet'] == p]
        ps = sum(1 for r in pr if r['success'])
        avg = sum(r['duration_sec'] for r in pr) / len(pr)
        print(f"{p}: {ps}/{len(pr)} ({ps/len(pr)*100:.1f}%) avg time: {avg:.1f}s")

    print(f"\n--- By Destination ---")
    for d in sorted(set(r['destination'] for r in results)):
        dr = [r for r in results if r['destination'] == d]
        ds = sum(1 for r in dr if r['success'])
        avg = sum(r['duration_sec'] for r in dr) / len(dr)
        print(f"{d}: {ds}/{len(dr)} ({ds/len(dr)*100:.1f}%) avg time: {avg:.1f}s")

    print(f"\n--- By Combination ---")
    combos = sorted(set((r['pallet'], r['destination']) for r in results))
    for p, d in combos:
        cr = [r for r in results if r['pallet'] == p and r['destination'] == d]
        cs = sum(1 for r in cr if r['success'])
        avg = sum(r['duration_sec'] for r in cr) / len(cr)
        print(f"{p}→{d}: {cs}/{len(cr)} ({cs/len(cr)*100:.1f}%) avg time: {avg:.1f}s")


def save_csv(results):
    filename = f"mission_6_results_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
    filepath = os.path.join(os.path.expanduser('~'), filename)
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['timestamp', 'pallet', 'destination', 'success', 'duration_sec'])
        writer.writeheader()
        writer.writerows(results)
    print(f"\nRaw data saved to: {filepath}")
    return filepath


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=1)
    parser.add_argument('--pallets', nargs='+', default=list(PALLETS.keys()))
    parser.add_argument('--destinations', nargs='+', default=list(DESTINATIONS.keys()))
    parser.add_argument('--obstacle-x', type=float, default=None)
    parser.add_argument('--obstacle-y', type=float, default=None)
    args = parser.parse_args()

    pallets = [p.upper() for p in args.pallets]
    destinations = [d.upper() for d in args.destinations]
    combos = [(p, d) for p in pallets for d in destinations]
    total = len(combos) * args.rounds

    if args.obstacle_x is not None:
        global OBSTACLE_X
        OBSTACLE_X = args.obstacle_x
    if args.obstacle_y is not None:
        global OBSTACLE_Y
        OBSTACLE_Y = args.obstacle_y

    no_obstacle = (args.obstacle_x is None and args.obstacle_y is None)

    print(f"=== Mission 6: Obstacle Avoidance Evaluation ===")
    print(f"Pallets: {pallets}")
    print(f"Destinations: {destinations}")
    print(f"Rounds: {args.rounds}")
    print(f"Total attempts: {total}")
    if no_obstacle:
        print(f"Obstacle: none")
    else:
        print(f"Obstacle: {OBSTACLE_SIZE}m box at ({OBSTACLE_X}, {OBSTACLE_Y})")

    init()
    if not no_obstacle:
        spawn_obstacle()

    results = []
    attempt = 0

    try:
        for round_num in range(1, args.rounds + 1):
            print(f"\n{'='*55}")
            print(f"Round {round_num}/{args.rounds}")
            for pallet, destination in combos:
                attempt += 1
                print(f"\n[{attempt}/{total}] {pallet} → {destination}")
                success, duration = run_mission(pallet, destination)
                results.append({
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'pallet': pallet,
                    'destination': destination,
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
        if not no_obstacle:
            remove_obstacle()

    print_report(results)
    save_csv(results)
    shutdown()


if __name__ == '__main__':
    main()
