# Mission Framework

This document covers the mission framework added in the `minsu/commander_test` branch.

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

The following visual markers have been added to `worlds/depot.sdf`.

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
| `raise_fork(duration)` | Raise the fork via cmd_vel |
| `lower_fork(duration)` | Lower the fork via cmd_vel |
| `wait_until_in_zone(cx, cy, radius)` | Wait until robot enters a zone, then cancel task |
| `shutdown()` | Shutdown ROS2 node |

### Running a Mission

```
cd ~/ros2_ws
source install/setup.bash
python3 src/wb_nav2_bringup/scripts/missions/mission_1.py
```

---

## Use Cases

| UC # | Title | Code File | Status |
|------|-------|-----------|--------|
| UC-01 | Bring robot to home | `missions/mission_1.py` | Done |
| UC-02 | Move Pallet P1 to D2 | `missions/mission_2.py` | In Progress |

---

## LLM Mission Control

The `execute_mission` tool has been added to `scripts/forklift_llm_control.py`.
This allows the LLM to run missions via natural language commands.

Example commands:
- "Run mission 1"
- "Execute mission 1"
- "Go home"

To start the LLM control session:
```
python3 ./install/wb_nav2_bringup/share/wb_nav2_bringup/scripts/forklift_llm_control.py
```
