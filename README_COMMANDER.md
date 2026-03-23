# Mission Framework

This document covers the mission framework for autonomous pallet transport.

## Post-Clone Setup (Fresh Install Notes)

After cloning, the following additional steps are required before running the code.

### 1. Install missing apt packages
```
sudo apt install -y ros-jazzy-apriltag-ros
sudo apt install -y ros-jazzy-image-proc
```

### 2. Install Ollama and download the LLM model
```
curl -fsSL https://ollama.com/install.sh | sh
ollama run qwen3.5:4b
```
When the model finishes downloading and a chat prompt appears, press `Ctrl+D` to exit.

### 3. Install Python packages for ROSA
```
python3 -m pip install jpl-rosa langchain-ollama langchain-anthropic langchain-openai python-dotenv langchain --break-system-packages
```

### 4. Fix import error in forklift_llm_control.py
Change line 3 of `scripts/forklift_llm_control.py`:
```python
# Before
from langchain.agents import tool

# After
from langchain_core.tools import tool
```

### 5. Correct dock id
The main README says to use `dock0` for docking, but the correct dock id is `dock1`.
See `configs/dock_database.yaml` for all available dock ids (`dock1` ~ `dock5`).

---

## World Markers

The following visual markers have been added to `worlds/mission_depot_v1.sdf`.

| Label | Type | Coordinates (x, y) | Description |
|-------|------|---------------------|-------------|
| S | Home | (-2.0, 0.0) | Robot starting/home zone (red border) |
| P1 | Pallet | dock1 position | Pallet pickup zone (blue border) |
| P2 | Pallet | dock2 position | Pallet pickup zone (blue border) |
| P3 | Pallet | dock3 position | Pallet pickup zone (blue border) |
| P4 | Pallet | dock4 position | Pallet pickup zone (blue border) |
| P5 | Pallet | dock5 position | Pallet pickup zone (blue border) |
| D1 | Drop zone | (-7.0, 0.0) | Pallet drop-off zone (green border) |
| D2 | Drop zone | (-5.0, 5.5) | Pallet drop-off zone (green border) |
| D3 | Drop zone | (-5.0, -5.5) | Pallet drop-off zone (green border) |

---

## Mission Framework

### Library: `scripts/pallet_mission.py`

Shared library used by all missions.

| Function | Description |
|----------|-------------|
| `init()` | Initialize ROS2 node and Nav2 |
| `go_home()` | Navigate to S zone |
| `go_to(x, y)` | Navigate to arbitrary map coordinates |
| `go_to_destination(dest_name)` | Navigate to D1, D2, or D3 |
| `dock(pallet_name)` | Dock to P1~P5 using AprilTag docking server |
| `backup(distance, speed)` | Back up after docking |
| `raise_fork(duration)` | Raise the fork via velocity_control |
| `lower_fork(duration)` | Lower the fork via velocity_control |
| `wait_until_in_zone(cx, cy, radius)` | Wait until robot enters a zone, then cancel task |
| `shutdown()` | Shutdown ROS2 node |

### Running a Mission

#### Mission 1: Bring robot to home (UC-01)

```
cd ~/ros2_ws
source install/setup.bash
python3 src/wb_nav2_bringup/scripts/missions/mission_1.py
```

The robot will navigate from its current position back to the home zone (S).

#### Mission 2: Move a pallet to a destination (UC-02)

Mission 2 moves a pallet from a pickup zone to a drop-off zone in 7 steps:
1. Dock to pallet using AprilTag detection
2. Raise fork
3. Navigate to destination
4. Lower fork
5. Back up (disengage from pallet)
6. Return home

```
cd ~/ros2_ws
source install/setup.bash
python3 src/wb_nav2_bringup/scripts/missions/mission_2.py <pallet> <destination>
```

| Argument | Options | Description |
|----------|---------|-------------|
| pallet | P1, P2, P3, P4, P5 | Pallet pickup zone |
| destination | D1, D2, D3 | Pallet drop-off zone |

Example:
```
python3 src/wb_nav2_bringup/scripts/missions/mission_2.py P1 D2
```

#### Running missions via LLM (ROSA)

First start the simulation, then in a second terminal start the LLM control session:
```
cd ~/ros2_ws
python3 ./install/wb_nav2_bringup/share/wb_nav2_bringup/scripts/forklift_llm_control.py
```

Then type natural language commands:
```
Execute mission 2 P1 D2
Execute mission 2 P2 D3
```

---

## Use Cases

| UC # | Title | Code File | Status |
|------|-------|-----------|--------|
| UC-01 | Bring robot to home | `missions/mission_1.py` | Done |
| UC-02 | Move pallet to destination | `missions/mission_2.py` | Done |

---

## LLM Mission Control

The `execute_mission` tool in `scripts/forklift_llm_control.py` allows the LLM to run missions via natural language commands.

Example commands:
- "Run mission 1"
- "Execute mission 2 P1 D2"
- "Execute mission 2 P2 D3"
