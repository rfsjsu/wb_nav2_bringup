# Set the workspace path here or as an exported environmental variable like this
#
# $>export WS_PATH="$HOME/ros2_ws
#
WB_PATH="$WS_PATH/install/wb_nav2_bringup/share/wb_nav2_bringup/"

ros2 launch wb_nav2_bringup tb4_simulation_launch.py \
    headless:=False \
    robot_name:="RX20_16" \
    robot_sdf:="$WB_PATH/urdf/RX20_16/main.xacro" \
    world:="$WB_PATH/worlds/depot.sdf" \
    map:="$WB_PATH/maps/depot.yaml" \
    params_file:="$WB_PATH/params/forklift_nav2_params.yaml" \
    bridge_config:="$WB_PATH/configs/forklift_bridge.yaml" \
    # -d -a

