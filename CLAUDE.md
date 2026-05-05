# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

Capstone scenario where two AMRs cooperate in Isaac Sim + ROS 2 Humble + Nav2 to "dig" target pallets out of warehouse rows. For any pallet line `<L>`, `PalletBin_<L>_01` is the blocker (in front), `PalletBin_<L>_02` is the target. AMR1 extracts the blocker → holds at stay_zone → AMR2 retrieves the target → AMR1 places the blocker into the former target slot. Goal is one standardized algorithm that works for any pallet line.

## Workspace layout (important)

- Project root: `/home/nsl/Capston_workspace/`
- The `amr_navigation/` package is **hardlinked** (same inode) to `/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/`. Editing either path updates both — but `colcon build` is invoked from the `humble_ws` side.
- `lift_cmd_file`/`lift_state_file` paths in `pallet_rules.yaml` point at the IsaacSim-ros_workspaces copy explicitly.

## File ownership (where to look)

- `amr_navigation/config/pallet_rules.yaml` — every tunable parameter (pre_dock_distance, lift_align_*, stay_align_*, amr2_wait_*, packing_station_*, log paths). Always check here first when asked about a threshold, gain, or distance.
- `amr_navigation/amr_navigation/capstone_logic.py` — geometry primitives only: `Pose3D`, `RobotSpec`, `PalletSpec`, `PreDockProcessResult`, `load_top_level_pallets`, `load_robot_specs`, `pre_dock_process_from_pair`. The shared pre_dock math lives here.
- `amr_navigation/amr_navigation/capstone_task_planner.py` — ~3800-line state-machine planner. Owns AMR1/AMR2 assignment, Nav2→manual handoff, all manual docking sub-phases, lift/reverse, stay_zone, PackingStation, replace flow. Edit here for transition logic; edit `pallet_rules.yaml` for numbers.
- `amr_navigation/amr_navigation/initial_pose_publisher.py` — publishes `/iw_hub_ros[_01]/initialpose` after waiting for subscribers.
- `amr_navigation/isaac_lift_bridge.py` — runs inside Isaac Sim Script Editor (NOT a ROS 2 node). Reads `lift_cmd.json`, drives `lift_joint`, writes `lift_state.json`.
- `occupancy_map/maps/Map01_robot_structure.txt` — AMR initial poses (`/iw_hub_ROS` at (7,−8,0.08), `/iw_hub_ROS_01` at (7,−10,0.08)).
- `occupancy_map/maps/Map01_prim_structure.txt` — pallet & PackingStation ground-truth poses. `_01`/`_02` follow the blocker/target convention.

## High-level architecture

Mission flow (sequence-critical):

1. AMR1: `MOVE_TO_PRE_DOCK` (Nav2) → `DIGGING` (manual approach) → `LIFTING_UP` → `TRANSPORTING` (loaded reverse to pre_dock) → `ALIGNING_TO_STAY` (rotate at pre_dock) → `MOVING_TO_STAY` (loaded reverse to stay_zone) → `WAIT` (at stay_zone)
2. AMR2: `WAIT_DELAYED_START` → `MOVE_TO_PRE_DOCK` → `APPROACHING_GATE` (manual creep at 3.5 m) → `WAIT_AT_GATE` (3 m stop) → `MOVE_TO_PRE_DOCK` (resume after AMR1 clears) → `DIGGING` → `LIFTING_UP` → `TRANSPORTING` → `MOVING_TO_PACKING` (Nav2) → `COMPLETED`
3. AMR1 relocation: `MOVE_TO_RELOCATION` (manual loaded stay_zone→pre_dock) → `RELOCATING` (loaded approach to former target slot) → `LOWERING` → `RETURNING` (unloaded reverse + home_backoff) → `MOVING_HOME` (Nav2) → `COMPLETED`

Mission state values are coarse-grained labels per robot stored in `_mission_state["robot_states"]`. The fine-grained step is in `_docking_state["phase"]` (e.g. `pre_turn_forward`, `dock`, `lift_align`, `settle`, `lift`, `reverse`, `stay_align`, `stay_reverse`, `move_to_relocation`, `amr1_home_backoff`).

Coordinate primitives (all derived at runtime, no hardcoding per line):

- **Shared pre_dock**: computed in `pre_dock_process_from_pair`. If blocker/target share y, pre_dock sits ±`pre_dock_distance` (= 2.0 m) on the opposite x side of the blocker from the target. Same logic on y axis if they share x.
- **AMR2 wait gate** (no fixed waypoint): cancels Nav2 at `amr2_wait_trigger_distance + amr2_wait_precancel_margin` (= 3.5 m), slow manual approach to `amr2_wait_trigger_distance` (= 3.0 m), resumes when AMR1 in WAIT and ≥ `amr1_clear_distance_for_amr2` (= 1.0 m) from pre_dock.
- **stay_zone**: `_compute_amr1_retreat_pose` — pre_dock + perpendicular offset by `amr1_retreat_distance` (= 2.0 m), away from AMR2's side.
- **PackingStation**: `load_named_prim_pose("/Root/PackingStation")` from prim file, falls back to YAML `packing_station_x/y`.

Manual docking sub-phases (dispatched in `_docking_step`):
`pre_turn_forward → start_settle → dock → lift_align → settle → lift → reverse` then branches: `stay_align → stay_reverse` (AMR1 to stay_zone), packing handoff (AMR2 to Nav2), `amr1_home_backoff` (AMR1 home), `move_to_relocation` (manual stay_zone→pre_dock for AMR1 replace).

Two-AMR safety: replace flow pauses if AMR1 (in `move_to_relocation`) and AMR2 (going to PackingStation) come within `amr1_replace_safety_stop_distance` (1.3 m); resumes at 1.7 m. Inline in `_coordination_step`.

## Common commands

Each terminal needs the standard environment first:
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
```

**Build (after editing planner/logic):**
```bash
cd ~/IsaacSim-ros_workspaces/humble_ws
colcon build --packages-select amr_navigation
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
```
YAML changes do NOT require rebuild (params loaded at launch).

**Run sequence (4 terminals):**
1. Isaac Sim with ROS 2 bridge: `~/isaacsim/isaac-sim.sh --enable isaacsim.ros2.bridge`
2. In Isaac Sim Script Editor: run `isaac_lift_bridge.py` (driver for `lift_joint`).
3. Nav2 + RViz: `ros2 launch iw_hub_navigation multiple_robot_iw_hub_navigation.launch.py map:=/home/nsl/Capston_workspace/occupancy_map/maps/Map01.yaml use_rviz:=True`
4. Planner: `ros2 launch amr_navigation amr_navigation.launch.py`

**Restart cleanly when stale processes interfere:**
```bash
pkill -9 -f component_container
pkill -9 -f rviz2
ros2 daemon stop && sleep 3 && ros2 daemon start
```

## Logs and reports

- `iw_hub_ros_log`, `iw_hub_ros_01_log` (workspace root) — per-robot phase tracking, ~1 line/sec. **First place to look when an AMR is stuck.**
- `/home/nsl/IsaacSim-ros_workspaces/result/result_log`, `latest_result.md`, `latest_result.json` — mission report.
- Lift bridge IPC: `/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/lift_{cmd,state}.json`.

When the user says "AMR is stuck at X", grep the per-robot log for the most recent `Manual docking state: phase=...` and `state=...` lines, then read the corresponding `_<phase>_step` in `capstone_task_planner.py` for the transition gate.

## Multi-robot Nav2 / Isaac Sim TF gotchas

These are NOT obvious from code and have wasted significant debugging time:

- Nav2 multi-robot launch auto-remaps `-r /tf:=tf -r /tf_static:=tf_static`. Inside namespace `/iw_hub_ros_01`, this means Nav2 subscribes to `/iw_hub_ros_01/tf`. **Isaac Sim TF Action Graph publishers must use RELATIVE topic names** (`tf`, `tf_static` — no leading slash) so they land in the same namespaced topic.
- In the `transform_tree_odometry` Action Graph: the dynamic transform node (`ros2_publish_raw_transform_tree`) must have `staticPublisher: ☐ unchecked`. The static transform nodes (`tf_tree_base_link_to_chassis`, `tf_tree_base_link_to_sensors`) must have `staticPublisher: ☑ checked`. Mixing these up causes `/tf` to be 0 Hz with publishers registered.
- Lidar `frameId` (in the `publish_*_2d_lidar_scan` nodes) must match a frame the static transform tree actually publishes (e.g., `Lidar_Front`, `Lidar_Rear`), not an arbitrary string. AMCL silently waits forever if the laser scan's `frame_id` is not in `tf_static`.
- nav2_params (`global_frame: odom`, `robot_base_frame: base_link`) stay UNPREFIXED — namespace isolation handles uniqueness, prefixing breaks the launch's auto-remap pattern.
- After Action Graph property changes (especially `staticPublisher` which affects QoS), Stop → Play in Isaac Sim is required for the publisher to be recreated. Stage save (Ctrl+S) is also required or changes revert.

## Working with the planner

- For new pallet lines, do NOT add line-specific conditionals. Verify `pre_dock_process_from_pair` returns a sensible result for the new line's blocker/target geometry; the planner is supposed to be line-agnostic.
- When changing transition logic, the affected method is almost always `_<phase>_step` or `_coordination_step`. Read both before editing.
- When tuning rotation/lift behavior, prefer YAML changes over code changes. The planner reads ~120 parameters from `pallet_rules.yaml`.
- `_compute_braked_angular_z` is shared between many phases via `_run_forward_in_place_rotation_step` / `_run_reverse_in_place_rotation_step`. Phase-specific behavior is injected via overrides passed in by each step function — be careful when changing this function, it's load-bearing across the whole state machine.
