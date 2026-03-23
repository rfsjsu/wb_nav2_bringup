#!/usr/bin/env python3
"""
Mission 1: Bring robot to home (S zone)
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from pallet_mission import init, go_home, shutdown

def main():
    init()
    print("=== Mission 1: Bring robot to home ===")
    go_home()
    print("=== Mission 1 complete! ===")
    shutdown()

if __name__ == '__main__':
    main()
