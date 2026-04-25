#!/usr/bin/env python3
"""
Mission 3: Evaluation - Repeated pallet transport to collect success rate data.

Usage:
    python3 mission_3.py [--rounds N] [--pallets P1 P2 ...] [--destinations D1 D2 ...]

Examples:
    python3 mission_3.py --rounds 1
    python3 mission_3.py --rounds 10
    python3 mission_3.py --rounds 5 --pallets P1 P2 --destinations D2
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

# Original pallet positions for reset
PALLET_ORIGINS = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.01, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.01, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.01, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.01, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.01, 'yaw': 0.0},
}

WORLD_NAME = 'mission_depot_v1'


def reset_all_pallets():
    for pallet_name in PALLET_ORIGINS:
        reset_pallet(pallet_name)


def reset_pallet(pallet_name):
    origin = PALLET_ORIGINS[pallet_name]
    model_name = f'pallet_{pallet_name[1]}'
    print(f"Resetting {model_name} to origin...")
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
    print(f"{model_name} reset complete!")


def run_mission(pallet, destination):
    start_time = time.time()
    try:
        print(f"\n--- Docking to {pallet} ---")
        if not dock(pallet):
            print("Docking failed, returning home...")
            lower_fork()
            go_home()
            reset_pallet(pallet)
            return False, time.time() - start_time

        raise_fork()
        if not check_pallet_lifted(pallet):
            print("Pallet not lifted correctly, returning home...")
            lower_fork()
            go_home()
            reset_pallet(pallet)
            return False, time.time() - start_time
        print(f"--- Navigating to {destination} ---")
        go_to_destination(destination)

        print(f"--- Lowering fork ---")
        lower_fork()

        print(f"--- Backing up ---")
        backup()

        print(f"--- Going home ---")
        go_home()

        duration = time.time() - start_time
        success = check_pallet_at_destination(pallet, destination)
        return success, duration

    except Exception as e:
        duration = time.time() - start_time
        print(f"Mission failed with exception: {e}")
        lower_fork()
        go_home()
        reset_pallet(pallet)
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


def save_csv(results):
    filename = f"mission_results_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
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
    total = len(combos) * args.rounds
    print(f"=== Mission 3: Evaluation ===")
    print(f"Pallets: {pallets}")
    print(f"Destinations: {destinations}")
    print(f"Rounds: {args.rounds}")
    print(f"Total attempts: {total}")

    init()

    results = []
    attempt = 0

    for round_num in range(1, args.rounds + 1):
        print(f"\n{'='*50}")
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
            time.sleep(2.0)
            if is_robot_out_of_bounds():
                print("Robot is out of map bounds! Aborting all missions.")
                raise SystemExit(1)

    print_report(results)
    save_csv(results)
    shutdown()


if __name__ == '__main__':
    main()
