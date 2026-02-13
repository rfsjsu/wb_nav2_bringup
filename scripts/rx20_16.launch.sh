WB_PATH="$HOME/forklift_project/wb_ws/install/wb_nav2_bringup/share/wb_nav2_bringup/"

ros2 launch wb_nav2_bringup tb4_simulation_launch.py \
    headless:=False \
    robot_name:="RX20_16" \
    robot_sdf:="$WB_PATH/urdf/RX20_16/main.xacro" \
    world:="$WB_PATH/worlds/depot.sdf" \
    map:="$WB_PATH/maps/depot.yaml" \
    params_file:="$WB_PATH/params/nav2_params.yaml" \
    bridge_config:="$WB_PATH/configs/forklift_bridge.yaml" \
    -d -a

