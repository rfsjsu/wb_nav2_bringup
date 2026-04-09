import subprocess
import time
import rclpy
from rclpy.node import Node

def shutdown_rviz():
    # Execute the ROS 2 command to kill the node
    subprocess.run(["pkill", "rviz2"])

def shutdown_gazebo():
    # Execute the ROS 2 command to kill the node
    subprocess.run(["pkill", "ruby"])

def run_rx20_16():
    process = subprocess.Popen(
        ['bash', './src/wb_nav2_bringup/scripts/rx20_16.launch.sh'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    return process

def run_pallet_test():
    process = subprocess.run(
        ["python",
         "./install/wb_nav2_bringup/share/wb_nav2_bringup/scripts/commander/pallet_docking.py"]
    )
    return process

if __name__ == '__main__':

    num_trials = 10
    for trial_idx in range(num_trials+1):
        print("Starting forklift simulation")
        process = run_rx20_16()
        # shutdown_rviz()

        bootup_time = 20
        print(f"Waiting {bootup_time} seconds before pallet trial {trial_idx}.")
        for i in range(bootup_time):
            time.sleep(1)
            print('.', end="", flush=True)
        print()

        print(f"====== Start Trial: {trial_idx} ======")
        start = time.perf_counter()
        run_pallet_test()
        end = time.perf_counter()
        print(f"Runtime: {end - start:.6f} seconds")
        print(f"====== End Trial: {trial_idx} ======")

        time.sleep(1)

        if(trial_idx < num_trials):
            print("Shutting down RViz")
            shutdown_rviz()
            print("Shutting down Gazebo")
            shutdown_gazebo()

            shutdown_time = 15
            print(f"Waiting {shutdown_time} seconds for all processes to end.")
            for i in range(shutdown_time):
                time.sleep(1)
                print('.', end="", flush=True)
            print()
    exit()
