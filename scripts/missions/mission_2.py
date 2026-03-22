#!/usr/bin/env python3
"""
Mission 2: Move a pallet from a pickup zone to a drop zone.

Usage:
    python3 mission_2.py <pallet> <destination>
    python3 mission_2.py P1 D2

Steps:
  1. Navigate to pallet staging area    <-- current step
  2. Dock to pallet (AprilTag docking)
  3. Raise fork
  4. Navigate to destination
  5. Lower fork
  6. Backup
  7. Go home
"""
import sys
sys.path.append('/home/minsu/ros2_ws/src/wb_nav2_bringup/scripts')
from pallet_mission import init, go_to, go_to_destination, dock, raise_fork, lower_fork, backup, go_home, shutdown, PALLETS, DESTINATIONS
from pallet_mission import DOCK_STAGING  # staging positions per dock

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 mission_2.py <pallet> <destination>")
        print("  pallet:      P1 ~ P5")
        print("  destination: D1 ~ D3")
        sys.exit(1)

    pallet = sys.argv[1].upper()
    destination = sys.argv[2].upper()

    if pallet not in PALLETS:
        print(f"Unknown pallet: {pallet}. Choose from {list(PALLETS.keys())}")
        sys.exit(1)
    if destination not in DESTINATIONS:
        print(f"Unknown destination: {destination}. Choose from {list(DESTINATIONS.keys())}")
        sys.exit(1)

    init()
    print(f"=== Mission 2: Move {pallet} to {destination} ===")

    # Step 1: Navigate to pallet staging area (handled by docking server)
    # Step 2: Dock to pallet
    print(f"--- Step 2: Docking to {pallet} ---")
    dock(pallet)
    print("--- Step 2 complete ---")

    # Step 3: Raise fork
    print("--- Step 3: Raising fork ---")
    raise_fork()
    print("--- Step 3 complete ---")

    # Step 4: Navigate to destination
    print(f"--- Step 4: Navigating to {destination} ---")
    go_to_destination(destination)
    print("--- Step 4 complete ---")

    # Step 5: Lower fork
    print("--- Step 5: Lowering fork ---")
    lower_fork()
    print("--- Step 5 complete ---")

    # Step 6: Backup
    print("--- Step 6: Backing up ---")
    backup()
    print("--- Step 6 complete ---")

    # Step 7: Go home
    print("--- Step 7: Going home ---")
    go_home()
    print("--- Step 7 complete ---")

    print(f"=== Mission 2 complete ===")
    shutdown()

if __name__ == '__main__':
    main()
