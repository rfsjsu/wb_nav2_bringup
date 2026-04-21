#!/usr/bin/env python3
"""
Mission 4: Evaluation - Repeated pallet docking with variable pallet orientations relative
to the dock pose (pallet_origins).  This is to test the scenarios where the pallet is not
perfectly aligned with it's assumed or pre-programmed position.

Here the yaw angle of the pallet is varied relative to the dock pose set in the docking server.

Usage:
    python3 mission_r04.py [--rounds N] [--pallets P1 P2 ...] [--destinations D1 D2 ...]

Examples:
    python3 mission_r04.py --rounds 1
    python3 mission_r04.py --rounds 10
    python3 mission_r04.py --rounds 5 --pallets P1 P2 --destinations D2
"""
import sys
import os
import time
import argparse
import csv
import subprocess
import math
import numpy as np
from datetime import datetime

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from pallet_mission import (
    init, go_to, go_to_destination, dock, raise_fork, lower_fork, fork_up, fork_down, set_gz_view,
    backup, spin, go_home, shutdown, pallet_movement_from_origin, PALLETS, DESTINATIONS
)

WORLD_NAME = 'mission_depot_v2'

# Original pallet positions for reset
PALLET_ORIGINS = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.01, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.01, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.01, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.01, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.01, 'yaw': 0.0},
}

PALLET_FIXED = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.01, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.01, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.01, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.01, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.01, 'yaw': 0.0},
}

ANGLES = np.arange(10.0, -12.0, -2.0)

# This will sort by increasing displacement regardless of sign.
indicies = np.argsort(np.abs(ANGLES))
ANGLES = ANGLES[indicies]

def pallet_rotation(degree):
    rad = (degree/360.0) * 2 * math.pi
    for pallet in PALLET_ORIGINS:
        PALLET_ORIGINS[pallet]['yaw'] = PALLET_FIXED[pallet]['yaw'] + rad
    return


def reset_pallet(pallet_name, debug=False):
    origin = PALLET_ORIGINS[pallet_name]
    model_name = f'pallet_{pallet_name[1]}'
    if(debug): print(f"Resetting {model_name} to origin...")
    # yaw to quaternion: z = sin(yaw/2), w = cos(yaw/2)
    import math
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
    time.sleep(2.0)
    if(debug): print(f"{model_name} reset complete!")


def run_mission(pallet, destination, debug=False):
    start_time = time.time()
    try:
        if(debug): print(f"--- Lowering fork ---")
        fork_down() #lower_fork()
        if(debug): print(f"\n--- Docking to {pallet} ---")
        status = dock(pallet, timeout=60.0)

        if(destination != 'NONE'):
            if(debug): print(f"--- Raising fork ---")
            fork_up() #raise_fork()

            if(debug): print(f"--- Navigating to {destination} ---")
            go_to_destination(destination)

            if(debug): print(f"--- Lowering fork ---")
            fork_down() #lower_fork()

        if(debug): print(f"--- Backing up ---")
        backup()

        if(debug): print(f"--- Turning around ---")
        spin()

        if(debug): print(f"--- Going home ---")
        go_home()

        duration = time.time() - start_time
        dist = pallet_movement_from_origin(pallet, PALLET_ORIGINS)
        success = status #check_pallet_at_destination(pallet, destination)
        return success, dist, duration

    except Exception as e:
        duration = time.time() - start_time
        print(f"Mission failed with exception: {e}")
        return False, duration


def print_report(results):
    print("\n" + "="*50)
    print("=== Forklift Mission Evaluation Report ===")
    print(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print("="*50)

    total = len(results)
    successes = sum(1 for r in results if r['success'])
    print(f"Total attempts: {total}")
    print(f"Total successes: {successes}")
    print(f"Overall success rate: {successes/total*100:.1f}%")

    print("\n--- By Pallet ---")
    for p in sorted(set(r['pallet'] for r in results)):
        p_results = [r for r in results if r['pallet'] == p]
        p_success = sum(1 for r in p_results if r['success'])
        avg_time = sum(r['duration_sec'] for r in p_results) / len(p_results)
        print(f"{p}: {p_success}/{len(p_results)} ({p_success/len(p_results)*100:.1f}%) avg time: {avg_time:.1f}s")

    print("\n--- By Destination ---")
    for d in sorted(set(r['destination'] for r in results)):
        d_results = [r for r in results if r['destination'] == d]
        d_success = sum(1 for r in d_results if r['success'])
        avg_time = sum(r['duration_sec'] for r in d_results) / len(d_results)
        print(f"{d}: {d_success}/{len(d_results)} ({d_success/len(d_results)*100:.1f}%) avg time: {avg_time:.1f}s")

    print("\n--- By Combination ---")
    combos = sorted(set((r['pallet'], r['destination']) for r in results))
    for p, d in combos:
        c_results = [r for r in results if r['pallet'] == p and r['destination'] == d]
        c_success = sum(1 for r in c_results if r['success'])
        avg_time = sum(r['duration_sec'] for r in c_results) / len(c_results)
        print(f"{p}→{d}: {c_success}/{len(c_results)} ({c_success/len(c_results)*100:.1f}%) avg time: {avg_time:.1f}s")

def save_csv(results, filepath=None, debug=False):
    '''
    This is tailored for reporting pallet docking success.
    Success is the return code from the docking server.
    Distance is the distance the pallet was moved.  If the fork is inserted perfectly, the pallet
    should not be disturbed and distance = 0.

    First time in create the file and return the filename. Second time in, use the filename
    to append to the same file.
    '''
    if(filepath == None):
        filename = f"mission_results_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
        filepath = os.path.join(os.path.expanduser('~'), 'mission_output', filename)
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['timestamp', 'pallet', 'destination', 'success', 'angle', 'distance', 'duration_sec'])
            writer.writeheader()
            writer.writerows(results)
    else:
        with open(filepath, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['timestamp', 'pallet', 'destination', 'success', 'angle', 'distance', 'duration_sec'])
            writer.writerows(results)        
    if(debug): print(f"\nRaw data saved to: {filepath}")
    return filepath


def main():
    output_filepath = None

    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=1)
    parser.add_argument('--pallets', nargs='+', default=list(PALLETS.keys()))
    parser.add_argument('--destinations', nargs='+', default=list(DESTINATIONS.keys()))
    args = parser.parse_args()

    pallets = [p.upper() for p in args.pallets]
    destinations = [d.upper() for d in args.destinations]

    for p in pallets:
        if p not in PALLETS:
            print(f"Unknown pallet: {p}")
            sys.exit(1)
    for d in destinations:
        if d not in DESTINATIONS:
            print(f"Unknown destination: {d}")
            sys.exit(1)

    combos = [(p, d) for p in pallets for d in destinations]
    total = len(combos) * len(ANGLES) * args.rounds
    print(f"=== Mission 3: Evaluation ===")
    print(f"Pallets: {pallets}")
    print(f"Destinations: {destinations}")
    print(f"Angels: {ANGLES}")
    print(f"Rounds: {args.rounds}")
    print(f"Total attempts: {total}")

    set_gz_view()
    init()

    results = []
    attempt = 0
    for round_num in range(1, args.rounds + 1):
        print(f"\n{'='*50}")
        print(f"Round {round_num}/{args.rounds}")
        for angle in ANGLES:
            pallet_rotation(angle)
            for pallet in pallets:
                reset_pallet(pallet)
            print(f"=== Pallet Angle: {angle} ===")
            for pallet, destination in combos:
                attempt += 1
                print(f"\n[{attempt}/{total}] {pallet} → {destination}")
                success, distance, duration = run_mission(pallet, destination)
                results.append({
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'pallet': pallet,
                    'destination': destination,
                    'angle': f"{angle:.1f}",
                    'distance': f"{distance:.4f}",
                    'success': success,
                    'duration_sec': round(duration, 1)
                })
                print(f"Result: {'SUCCESS' if success else 'FAILED'} ({duration:.1f}s)")
                reset_pallet(pallet)
                time.sleep(2.0)
            output_filepath = save_csv(results, output_filepath)
            results = []
    shutdown()


if __name__ == '__main__':
    main()
