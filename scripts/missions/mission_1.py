#!/usr/bin/env python3
"""
Mission 1: Bring robot to home (S zone)
"""
import sys
sys.path.append('/home/minsu/ros2_ws/src/wb_nav2_bringup/scripts')
from pallet_mission import init, go_home, shutdown

def main():
    init()
    print("=== Mission 1: Bring robot to home ===")
    go_home()
    print("=== Mission 1 complete! ===")
    shutdown()

if __name__ == '__main__':
    main()
