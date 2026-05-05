# AMR Navigation

This folder is now structured as a ROS 2 Python package.

## What is included

- `package.xml`: ROS 2 package metadata
- `setup.py`, `setup.cfg`: Python package install settings
- `amr_navigation/`: Python module for navigation logic
- `launch/`: ROS 2 launch files
- `config/`: parameter files
- `resource/`: ament package marker

## Current entry point

- `capstone_task_planner`

This node is a starter point for the capstone task sequencing logic.
It now includes:

- top-level `PalletBin_*` prim loading from `Map01_prim_structure.txt`
- target-to-blocker mapping based on the `*_02 -> *_01` naming rule
- spawn information for `/iw_hub_ROS` and `/iw_hub_ROS_01`
- closest-AMR selection for the blocker pallet
