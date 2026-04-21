#!/usr/bin/env python3
"""
Mission 4: Speed Evaluation - Find the maximum reliable speed.

Gradually increases robot speed and measures success rate at each speed level.
Stops when error rate exceeds the threshold.

Usage:
    python3 mission_4.py [--speed-start 0.5] [--speed-end 3.0] [--speed-step 0.5] [--rounds 3] [--stop-error-rate 0.2]

Examples:
    python3 mission_4.py --rounds 3
    python3 mission_4.py --speed-start 0.5 --speed-end 2.0 --speed-step 0.5 --rounds 3 --stop-error-rate 0.2
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

PARAMS_FILE = os.path.expanduser('~/ros2_ws/src/wb_nav2_bringup/params/forklift_nav2_params.yaml')

PALLET_ORIGINS = {
    'P1': {'x': -10.0, 'y': -6.0,  'z': 0.01, 'yaw': 0.0},
    'P2': {'x': -14.0, 'y': -3.0,  'z': 0.01, 'yaw': 1.5708},
    'P3': {'x': -14.0, 'y':  0.0,  'z': 0.01, 'yaw': 1.5708},
    'P4': {'x': -14.0, 'y':  3.0,  'z': 0.01, 'yaw': 1.5708},
    'P5': {'x': -10.0, 'y':  6.0,  'z': 0.01, 'yaw': 0.0},
}

WORLD_NAME = 'mission_depot_v1'


def set_speed(speed):
    print(f"Setting speed to {speed} m/s...")
    # Update yaml for persistence
    with open(PARAMS_FILE, 'r') as f:
        content = f.read()
    content = re.sub(r'(vx_max:\s*)[\d.]+', f'\\g<1>{speed}', content)
    content = re.sub(r'(max_velocity:\s*\[)[\d.]+', f'\\g<1>{speed}', content)
    with open(PARAMS_FILE, 'w') as f:
        f.write(content)
    # Apply dynamically via ros2 param set
    subprocess.run(['ros2', 'param', 'set', '/velocity_smoother', 'max_velocity', f'[{speed}, 0.0, 2.0]'], capture_output=True)
    subprocess.run(['ros2', 'param', 'set', '/controller_server', 'FollowPath.vx_max', str(speed)], capture_output=True)
    time.sleep(1.0)
    print(f"Speed set to {speed} m/s")


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
        go_to_destination(destination)
        lower_fork()
        backup()
        go_home()
        duration = time.time() - start_time
        success = check_pallet_at_destination(pallet, destination)
        return success, duration
    except Exception as e:
        duration = time.time() - start_time
        print(f"Mission failed with exception: {e}")
        return False, duration


def print_report(results, stop_error_rate):
    print("\n" + "="*55)
    print("=== Forklift Mission 4 - Speed Evaluation Report ===")
    print(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print(f"Stop condition: error rate > {stop_error_rate*100:.0f}%")
    print("="*55)

    total = len(results)
    successes = sum(1 for r in results if r['success'])
    print(f"\n--- Overall ---")
    print(f"Total attempts: {total}")
    print(f"Total successes: {successes}")
    print(f"Overall success rate: {successes/total*100:.1f}%")

    print(f"\n--- By Speed ---")
    speeds = sorted(set(r['speed'] for r in results))
    max_reliable_speed = None
    for s in speeds:
        sr = [r for r in results if r['speed'] == s]
        ss = sum(1 for r in sr if r['success'])
        avg = sum(r['duration_sec'] for r in sr) / len(sr)
        rate = ss/len(sr)
        stopped = " ← stopped here" if rate <= (1 - stop_error_rate) else ""
        print(f"Speed {s} m/s: {ss}/{len(sr)} ({rate*100:.1f}%) avg time: {avg:.1f}s{stopped}")
        if rate >= (1 - stop_error_rate):
            max_reliable_speed = s
    print(f"\n--- Critical Speed ---")
    print(f"Max reliable speed: {max_reliable_speed} m/s (success rate > {(1-stop_error_rate)*100:.0f}%)")

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

    print(f"\n--- By Speed + Pallet ---")
    for s in speeds:
        row = f"Speed {s} m/s |"
        for p in sorted(set(r['pallet'] for r in results)):
            pr = [r for r in results if r['speed'] == s and r['pallet'] == p]
            if pr:
                ps = sum(1 for r in pr if r['success'])
                row += f" {p}: {ps}/{len(pr)}"
        print(row)


def save_csv(results):
    filename = f"mission_4_results_{datetime.now().strftime('%Y-%m-%d_%H-%M')}.csv"
    filepath = os.path.join(os.path.expanduser('~'), filename)
    with open(filepath, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['timestamp', 'speed', 'pallet', 'destination', 'success', 'duration_sec'])
        writer.writeheader()
        writer.writerows(results)
    print(f"\nRaw data saved to: {filepath}")
    return filepath


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed-start', type=float, default=0.5)
    parser.add_argument('--speed-end', type=float, default=3.0)
    parser.add_argument('--speed-step', type=float, default=0.5)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--stop-error-rate', type=float, default=0.2)
    args = parser.parse_args()

    pallets = list(PALLETS.keys())
    destinations = list(DESTINATIONS.keys())
    combos = [(p, d) for p in pallets for d in destinations]

    speeds = []
    s = args.speed_start
    while s <= args.speed_end + 0.001:
        speeds.append(round(s, 2))
        s += args.speed_step

    print(f"=== Mission 4: Speed Evaluation ===")
    print(f"Speeds: {speeds}")
    print(f"Rounds per speed: {args.rounds}")
    print(f"Stop condition: error rate > {args.stop_error_rate*100:.0f}%")

    init()
    results = []

    for speed in speeds:
        set_speed(speed)
        print(f"\n{'='*55}")
        print(f"Testing speed: {speed} m/s")

        speed_results = []
        for round_num in range(1, args.rounds + 1):
            print(f"\n  Round {round_num}/{args.rounds}")
            for pallet, destination in combos:
                print(f"  {pallet} → {destination}")
                success, duration = run_mission(pallet, destination)
                result = {
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'speed': speed,
                    'pallet': pallet,
                    'destination': destination,
                    'success': success,
                    'duration_sec': round(duration, 1)
                }
                results.append(result)
                speed_results.append(result)
                print(f"  Result: {'SUCCESS' if success else 'FAILED'} ({duration:.1f}s)")
                reset_pallet(pallet)
                time.sleep(2.0)

        # Check error rate for this speed
        error_rate = 1 - sum(1 for r in speed_results if r['success']) / len(speed_results)
        print(f"\nSpeed {speed} m/s error rate: {error_rate*100:.1f}%")
        if error_rate > args.stop_error_rate:
            print(f"Error rate exceeded {args.stop_error_rate*100:.0f}% — stopping!")
            break

    print_report(results, args.stop_error_rate)
    save_csv(results)

    # Restore original speed
    set_speed(0.5)
    print("Speed restored to 0.5 m/s")
    shutdown()


if __name__ == '__main__':
    main()
