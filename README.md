# Capston Workspace

This workspace is the top-level folder for the capstone scenario.

## Goal

Two AMRs cooperate to move target pallets.

Before moving a target pallet, the AMRs must first move the blocking pallet in front of it.

## Pallet Naming Rule

- `PalletBin_A_01`
- `PalletBin_A_02`
- `PalletBin_B_01`
- `PalletBin_B_02`

Pallet bins ending with `01` are treated as obstacle pallets.

## Folder Layout

- `occupancy_map/`: static map files and map-related configuration
- `amr_navigation/`: AMR movement logic, launch files, and navigation configuration
- `environment/`: Isaac Sim USD environment files

## ROS 2 Build Integration

The `amr_navigation/` package is linked into the existing ROS 2 workspace here:

- `humble_ws/src/capstone/amr_navigation`

This means you can keep capstone files organized under `Capston_workspace/` while still building from the existing `humble_ws` workspace.

## Current Files

- Environment USD: `environment/usd/Capston_Map01_RC.usd`
- Occupancy map image: `occupancy_map/maps/Capston_Map01_RC.png`
- Occupancy map yaml: `occupancy_map/maps/Map01.yaml`

## Run Guide

터미널 1: Isaac Sim

Role: open Isaac Sim with ROS 2 bridge and run the simulation stage.

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
~/isaacsim/isaac-sim.sh --enable isaacsim.ros2.bridge
```



# 리프트 연결
isaac_lift_bridge - script editor로 실행


터미널 3: iw_hub_navigation 다시 빌드
역할: 방금 Nav2 controller 수정 반영

cd ~/IsaacSim-ros_workspaces/humble_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_navigation
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash


터미널 2: Nav2 + RViz 다시 실행
역할: 수정된 controller로 Nav2 실행

기존 거 켜져 있으면 Ctrl+C로 끄고 다시 실행하세요.

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
ros2 launch iw_hub_navigation multiple_robot_iw_hub_navigation.launch.py map:=/home/nsl/Capston_workspace/occupancy_map/maps/Map01.yaml use_rviz:=True

터미널 4: amr_navigation 다시 실행
역할: 자동 initial pose + planner 실행

기존 거 켜져 있으면 Ctrl+C로 끄고 다시 실행하세요.

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
ros2 launch amr_navigation amr_navigation.launch.py



source /opt/ros/humble/setup.bash
source ~/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash

Mission state 세분화 (총 17곳):

AMR1: MOVE_TO_PRE_DOCK → DIGGING → LIFTING_UP → TRANSPORTING → ALIGNING_TO_STAY → MOVING_TO_STAY → WAIT → MOVE_TO_RELOCATION → RELOCATING → LOWERING → RETURNING → MOVING_HOME → COMPLETED

AMR2: WAIT_DELAYED_START → MOVE_TO_PRE_DOCK → APPROACHING_GATE → WAIT_AT_GATE → MOVE_TO_PRE_DOCK → DIGGING → LIFTING_UP → TRANSPORTING → MOVING_TO_PACKING → COMPLETED


## Version History

- version 2
  - 수정사항: 제자리 회전속도 높임 (최소 0.3 -> 0.4)
  - lift driver 값 상승 (100000 -> 200000)
  - AMR2 출발 지연 변경 (10초 -> 20초)
