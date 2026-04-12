import subprocess
import time
import rclpy
from rclpy.node import Node
import json

def shutdown_rviz():
    # Execute the ROS 2 command to kill the node
    subprocess.run(["pkill", "rviz2"])

def shutdown_gazebo():
    # Execute the ROS 2 command to kill the node
    subprocess.run(["pkill", "ruby"])

def get_pose(name):
    '''
    Get the pose of a named object by reading a Gazebo topic directly.
    This code is intended to get the pose of an object at rest.
    Going directly to Gazebo eliminates the need to create a ROS2 bridge.
    We have the try loop because on rare occasions the returned json
    is malformed.
    '''
    ret = ''
    while True:
        try:
            result = subprocess.run(
                [
                    'gz',
                    'topic',
                    '-e',
                    '--json-output',
                    '-n',
                    '1',
                    '-t',
                    '/world/depot/pose/info'
                ],
                capture_output=True,
                text=True
            )
            data = json.loads(result.stdout)
            for pose in data['pose']:
                if(pose['name'] == name):
                    ret = pose
            break
        except json.decoder.JSONDecodeError:
            print("Get pose failed with JSONDecodeError. Trying again.")
    return ret

def run_rx20_16():
    process = subprocess.Popen(
        ['bash', './src/wb_nav2_bringup/scripts/rx20_16.launch.sh'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    return process

def run_pallet_test(x,y):
    process = subprocess.run(
        ["python",
         "./install/wb_nav2_bringup/share/wb_nav2_bringup/scripts/commander/pallet_docking.py",
         str(x),
         str(y)]
    )
    return process

if __name__ == '__main__':

    (x,y) = (-8.0, 1.5)

    results = []

    home_y = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0]
    home_x = [-6.0, -8.0, -10.0, -12.0]

    # home_y = [-1.0]
    # home_x = [-8.0]

    trial_idx = 0
    # for trial_idx in range(num_trials+1):
    for x in home_x:
        for y in home_y:
            trial_data = {}
            trial_data['home'] = [x,y]

            print("Starting forklift simulation")
            process = run_rx20_16()

            bootup_time = 20
            print(f"Waiting {bootup_time} seconds before pallet trial {trial_idx}.")
            for i in range(bootup_time):
                time.sleep(1)
                print('.', end="", flush=True)
            print()

            print(f"====== Start Trial: {trial_idx} ======")
            start = time.perf_counter()
            trial_data['pallet_2_start'] = get_pose('pallet_2')

            run_pallet_test(x,y)

            trial_data['pallet_2_end'] = get_pose('pallet_2')
            trial_data['forklift_end'] = get_pose('RX20_16')
            end = time.perf_counter()

            trial_data['run_time'] = end - start
            results.append(trial_data)
            print(f"====== End Trial: {trial_idx} ======")
  
            time.sleep(1)

            # if(trial_idx < num_trials):
            shutdown_rviz()
            shutdown_gazebo()

            shutdown_time = 15
            print(f"Waiting {shutdown_time} seconds for all processes to end.")
            for i in range(shutdown_time):
                time.sleep(1)
                print('.', end="", flush=True)
            print()
            trial_idx += 1

    # for data in results:
    #     print(data)

    with open('/home/rsf/data.json', 'w') as f:
        json.dump(results, f, indent=4)
    exit()
