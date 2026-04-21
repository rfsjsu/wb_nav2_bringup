#!/usr/bin/env python3
"""
Mission 5: Angular Velocity Evaluation - Find the maximum reliable docking angular velocity.

Gradually increases v_angular_max and measures docking success rate at each level.
Stops when error rate exceeds the threshold.

Usage:
    python3 mission_5.py [--angular-start 0.3] [--angular-end 2.0] [--angular-step 0.2] [--rounds 3] [--stop-error-rate 0.2]

Examples:
    python3 mission_5.py --rounds 3
    python3 mission_5.py --angular-start 0.3 --angular-end 2.0 --angular-step 0.2 --rounds 3 --stop-error-rate 0.2
"""
import sys
import os
import time
import argparse
import csv
import subprocess
import re
from datetime import datetime

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from pallet_mission import (
    init, go_to, go_to_destination, dock, raise_fork, lower_fork,
    backup, go_home, shutdown, check_pallet_at_destination, PALLETS, DESTINATIONS
)

PALLET_ORIGINS = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.01, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.01, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.01, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.01, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.01, 'yaw': 0.0},
}

WORLD_NAME = 'mission_depot_v1'


def set_angular_velocity(v_angular):
    print(f"Setting v_angular_max to {v_angular} rad/s...")
    subprocess.run([
        'ros2', 'param', 'set', '/docking_server',
        'controller.v_angular_max', str(v_angular)
    ], capture_output=True)
    time.sleep(1.0)
    print(f"v_angular_max set to {v_angular} rad/s")


def reset_pallet(pallet_name):
    import math
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
    time.sleep(2.0)


def run_mission(pallet, destination):
    start_time = time.time()
    try:
        if not dock(pallet):
            print("Docking failed, returning home...")
            go_home()
            return False, time.time() - start_time
        raise_fork()
        if not go_to_destination(destination):
            print("Navigation to destination failed, returning home...")
            lower_fork()
            go_home()
            return False, time.time() - start_time
        lower_fork()
        backup()
        go_home()
        duration = time.time() - start_time
        success = check_pallet_at_destination(pallet, destination)
        return success, duration
    except Exception as e:
        duration = time.time() - start_time
        print(f"Mission failed with exception: {e}")
        go_home()
        return False, duration


def print_report(results, stop_error_rate):
    print("\n" + "="*55)
    print("=== Forklift Mission 5 - Angular Velocity Evaluation ===")
    print(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"Stop condition: error rate > {stop_error_rate*100:.0f}%")
    print("="*55)

    total = len(results)
    successes = sum(1 for r in results if r['success'])
    print(f"\n--- Overall ---")
    print(f"Total attempts: {total}")
    print(f"Total successes: {successes}")
    print(f"Overall success rate: {successes/total*100:.1f}%")

    print(f"\n--- By Angular Velocity ---")
    angulars = sorted(set(r['v_angular_max'] for r in results))
    max_reliable = None
    for v in angulars:
        vr = [r for r in results if r['v_angular_max'] == v]
        vs = sum(1 for r in vr if r['success'])
        avg = sum(r['duration_sec'] for r in vr) / len(vr)
        rate = vs / len(vr)
        stopped = " ← stopped here" if rate <= (1 - stop_error_rate) else ""
        print(f"v_angular_max {v} rad/s: {vs}/{len(vr)} ({rate*100:.1f}%) avg time: {avg:.1f}s{stopped}")
        if rate >= (1 - stop_error_rate):
            max_reliable = v

    print(f"\n--- Critical Angular Velocity ---")
    print(f"Max reliable v_angular_max: {max_reliable} rad/s (success rate > {(1-stop_error_rate)*100:.0f}%)")

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

    print(f"\n--- By Angular Velocity + Pallet ---")
    for v in angulars:
        row = f"v_angular {v} |"
        for p in sorted(set(r['pallet'] for r in results)):
            pr = [r for r in results if r['v_angular_max'] == v and r['pallet'] == p]
            if pr:
                ps = sum(1 for r in pr if r['success'])
                row += f" {p}: {ps}/{len(pr)}"
        print(row)


def save_csv(results):
    filename = f"mission_5_results_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
    filepath = os.path.join(os.path.expanduser('~'), filename)
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['timestamp', 'v_angular_max', 'pallet', 'destination', 'success', 'duration_sec'])
        writer.writeheader()
        writer.writerows(results)
    print(f"\nRaw data saved to: {filepath}")
    return filepath


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--angular-start', type=float, default=0.3)
    parser.add_argument('--angular-end', type=float, default=2.0)
    parser.add_argument('--angular-step', type=float, default=0.2)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--stop-error-rate', type=float, default=0.2)
    args = parser.parse_args()

    pallets = list(PALLETS.keys())
    destinations = list(DESTINATIONS.keys())
    combos = [(p, d) for p in pallets for d in destinations]

    angulars = []
    v = args.angular_start
    while v <= args.angular_end + 0.001:
        angulars.append(round(v, 2))
        v += args.angular_step

    print(f"=== Mission 5: Angular Velocity Evaluation ===")
    print(f"v_angular_max range: {angulars}")
    print(f"Rounds per angular velocity: {args.rounds}")
    print(f"Stop condition: error rate > {args.stop_error_rate*100:.0f}%")

    init()
    results = []

    for v_angular in angulars:
        set_angular_velocity(v_angular)
        print(f"\n{'='*55}")
        print(f"Testing v_angular_max: {v_angular} rad/s")

        angular_results = []
        for round_num in range(1, args.rounds + 1):
            print(f"\n  Round {round_num}/{args.rounds}")
            for pallet, destination in combos:
                print(f"  {pallet} → {destination}")
                success, duration = run_mission(pallet, destination)
                result = {
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'v_angular_max': v_angular,
                    'pallet': pallet,
                    'destination': destination,
                    'success': success,
                    'duration_sec': round(duration, 1)
                }
                results.append(result)
                angular_results.append(result)
                print(f"  Result: {'SUCCESS' if success else 'FAILED'} ({duration:.1f}s)")
                reset_pallet(pallet)
                time.sleep(2.0)

        error_rate = 1 - sum(1 for r in angular_results if r['success']) / len(angular_results)
        print(f"\nv_angular_max {v_angular} rad/s error rate: {error_rate*100:.1f}%")
        if error_rate > args.stop_error_rate:
            print(f"Error rate exceeded {args.stop_error_rate*100:.0f}% — stopping!")
            break

    print_report(results, args.stop_error_rate)
    save_csv(results)

    # Restore original angular velocity
    set_angular_velocity(0.3)
    print("v_angular_max restored to 0.3 rad/s")
    shutdown()


if __name__ == '__main__':
    main()
