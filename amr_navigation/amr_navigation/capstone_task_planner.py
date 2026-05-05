from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import json
from nav2_msgs.action import NavigateToPose
import math
import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from amr_navigation.capstone_logic import (
    NAV2_NAMESPACE_BY_ROBOT,
    PRIM_STRUCTURE_FILE,
    ROBOT_STRUCTURE_FILE,
    Pose3D,
    PreDockProcessResult,
    blocker_name_from_target,
    load_named_prim_pose,
    load_robot_specs,
    load_top_level_pallets,
    pre_dock_process_from_pair,
    select_closest_robot,
)


class CapstoneTaskPlanner(Node):
    def __init__(self) -> None:
        super().__init__("capstone_task_planner")

        self.declare_parameter("target_pallet", "PalletBin_B_02")
        self.declare_parameter("prim_structure_file", PRIM_STRUCTURE_FILE)
        self.declare_parameter("robot_structure_file", ROBOT_STRUCTURE_FILE)
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("pre_dock_distance", 2.0)
        self.declare_parameter("pre_dock_position_handoff_tolerance", 0.1)
        self.declare_parameter("amr2_pre_dock_position_handoff_tolerance", 0.4)
        self.declare_parameter("axis_tolerance", 0.1)
        self.declare_parameter("amcl_ready_min_messages", 5)
        self.declare_parameter("amcl_ready_settle_sec", 3.0)
        self.declare_parameter("dock_distance", 1.0)
        self.declare_parameter("dock_speed", 0.12)
        self.declare_parameter("dock_ramp_duration_sec", 1.0)
        self.declare_parameter("dock_heading_gain", 1.5)
        self.declare_parameter("dock_heading_stop_tolerance", 0.2)
        self.declare_parameter("dock_initial_heading_skip_tolerance", math.radians(10.0))
        self.declare_parameter("stay_align_stop_tolerance", math.radians(6.0))
        self.declare_parameter("stay_align_heading_gain", 2.8)
        self.declare_parameter("stay_align_max_angular_speed", 3.0)
        self.declare_parameter("stay_align_brake_tolerance", 0.08)
        self.declare_parameter("stay_align_min_angular_speed", 0.6)
        self.declare_parameter("loaded_stay_align_rotation_speed_scale", 0.2)
        self.declare_parameter("loaded_stay_align_min_angular_speed", 0.1)
        self.declare_parameter("dock_unloaded_align_heading_gain", 3.0)
        self.declare_parameter("dock_unloaded_align_max_angular_speed", 3.0)
        self.declare_parameter("dock_align_timeout_sec", 2.0)
        self.declare_parameter("dock_start_settle_sec", 0.4)
        self.declare_parameter("loaded_dock_start_settle_sec", 1.0)
        self.declare_parameter("pre_turn_forward_distance", 0.2)
        self.declare_parameter("pre_turn_forward_speed", 0.12)
        self.declare_parameter("pre_turn_heading_stop_tolerance", math.radians(4.0))
        self.declare_parameter("pre_turn_heading_gain", 2.6)
        self.declare_parameter("pre_turn_max_angular_speed", 0.9)
        self.declare_parameter("pre_turn_brake_tolerance", 0.06)
        self.declare_parameter("pre_turn_min_angular_speed", 0.3)
        self.declare_parameter("loaded_pre_turn_rotation_speed_scale", 1.0)
        self.declare_parameter("loaded_pre_turn_min_angular_speed", 0.45)
        self.declare_parameter("angular_brake_start_tolerance", math.radians(20.0))
        self.declare_parameter("angular_brake_exponent", 1.7)
        self.declare_parameter("angular_brake_min_speed", 0.08)
        self.declare_parameter("angular_brake_final_tolerance", math.radians(10.0))
        self.declare_parameter("angular_brake_final_exponent", 4.2)
        self.declare_parameter("angular_brake_final_min_speed", 0.02)
        self.declare_parameter("dock_cross_track_heading_limit", math.radians(8.0))
        self.declare_parameter("dock_on_move_heading_gain", 0.9)
        self.declare_parameter("dock_on_move_max_angular_speed", 0.55)
        self.declare_parameter("dock_axis_follow_heading_limit", math.radians(4.0))
        self.declare_parameter("dock_axis_follow_heading_gain", 0.45)
        self.declare_parameter("dock_axis_follow_max_angular_speed", 0.22)
        self.declare_parameter("dock_near_heading_gain", 0.55)
        self.declare_parameter("dock_near_max_angular_speed", 0.22)
        self.declare_parameter("dock_near_cross_track_heading_limit", 0.08)
        self.declare_parameter("dock_min_linear_ratio", 0.7)
        self.declare_parameter("dock_creep_distance", 0.18)
        self.declare_parameter("dock_creep_speed", 0.08)
        self.declare_parameter("dock_heading_drive_tolerance", 0.35)
        self.declare_parameter("dock_heading_correction_tolerance", 0.12)
        self.declare_parameter("dock_cross_track_gain", 1.0)
        self.declare_parameter("dock_cross_track_heading_gain", 4.0)
        self.declare_parameter("dock_cross_track_tolerance", 0.05)
        self.declare_parameter("dock_cross_track_stop_tolerance", 0.02)
        self.declare_parameter("dock_line_acquire_distance", 0.3)
        self.declare_parameter("dock_axis_lock_distance", 0.3)
        self.declare_parameter("dock_max_overshoot", 0.4)
        self.declare_parameter("dock_max_angular_speed", 0.5)
        self.declare_parameter("dock_entry_offset", 0.2)
        self.declare_parameter("blocker_reach_tolerance", 0.18)
        self.declare_parameter("pre_lift_stop_duration_sec", 0.8)
        self.declare_parameter("lift_align_tolerance", 0.05)
        self.declare_parameter("lift_align_completion_tolerance", 0.005)
        self.declare_parameter("lift_align_deadband", 0.008)
        self.declare_parameter("lift_align_heading_gain", 2.4)
        self.declare_parameter("lift_align_heading_gain_max", 4.2)
        self.declare_parameter("lift_align_heading_gain_ramp_error", 0.18)
        self.declare_parameter("lift_align_max_angular_speed", 0.6)
        self.declare_parameter("lift_align_min_angular_speed", 0.4)
        self.declare_parameter("lift_align_loaded_min_angular_speed", 1.2)
        self.declare_parameter("lift_settle_sec", 1.0)
        self.declare_parameter("lift_precheck_position_tolerance", 0.01)
        self.declare_parameter("lift_precheck_yaw_tolerance", 0.02)
        self.declare_parameter("reverse_arrive_tolerance", 0.08)
        self.declare_parameter("reverse_speed", 0.12)
        self.declare_parameter("reverse_ramp_duration_sec", 1.0)
        self.declare_parameter("reverse_ramp_start_speed", 0.12)
        self.declare_parameter("reverse_brake_distance", 0.6)
        self.declare_parameter("reverse_brake_exponent", 1.7)
        self.declare_parameter("reverse_creep_speed", 0.12)
        self.declare_parameter("manual_terminal_brake_distance", 0.6)
        self.declare_parameter("manual_terminal_creep_speed", 0.18)
        self.declare_parameter("manual_terminal_final_brake_distance", 0.18)
        self.declare_parameter("manual_terminal_final_speed", 0.04)
        self.declare_parameter("amr2_packing_backoff_distance", 1.0)
        self.declare_parameter("pre_dock_wait_radius", 0.7)
        self.declare_parameter("amr1_retreat_distance", 2.0)
        self.declare_parameter("amr1_clear_distance_for_amr2", 1.0)
        self.declare_parameter("amr2_clear_distance_for_amr1_replace", 1.0)
        self.declare_parameter("amr2_packing_nav_min_travel_for_amr1_replace", 0.5)
        self.declare_parameter("amr1_replace_safety_stop_distance", 1.3)
        self.declare_parameter("amr1_replace_safety_resume_distance", 1.7)
        self.declare_parameter("amr2_start_delay_sec", 5.0)
        self.declare_parameter("amr2_wait_distance_before_pre_dock", 2.0)
        self.declare_parameter("amr2_wait_trigger_distance", 3.0)
        self.declare_parameter("amr2_wait_precancel_margin", 0.5)
        self.declare_parameter("amr2_wait_manual_approach_speed", 0.08)
        self.declare_parameter("amr2_wait_manual_approach_heading_gain", 1.2)
        self.declare_parameter("amr2_wait_manual_approach_max_angular_speed", 0.35)
        self.declare_parameter("amr2_wait_manual_approach_heading_tolerance", 0.18)
        self.declare_parameter("stay_zone_arrive_tolerance", 0.1)
        self.declare_parameter("stay_reverse_brake_distance", 0.6)
        self.declare_parameter("stay_reverse_creep_speed", 0.12)
        self.declare_parameter("packing_station_prim", "/Root/PackingStation")
        self.declare_parameter("packing_station_x", -9.056734933448762)
        self.declare_parameter("packing_station_y", -11.017332251450735)
        self.declare_parameter("packing_station_yaw", 0.0)
        self.declare_parameter("packing_align_tolerance", 0.12)
        self.declare_parameter("packing_align_heading_gain", 1.6)
        self.declare_parameter("packing_align_max_angular_speed", 0.35)
        self.declare_parameter("loaded_motion_scale", 0.7)
        self.declare_parameter("loaded_rotation_speed_scale", 0.6)
        self.declare_parameter("lift_joint_name", "lift_joint")
        self.declare_parameter("lift_target_position", 0.04)
        self.declare_parameter(
            "lift_cmd_file",
            "/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/lift_cmd.json",
        )
        self.declare_parameter(
            "lift_state_file",
            "/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/lift_state.json",
        )
        self.declare_parameter("lift_state_tolerance", 0.02)
        self.declare_parameter("lift_completion_tolerance", 0.001)
        self.declare_parameter("lift_lower_completion_tolerance", 0.02)
        self.declare_parameter("lift_verify_timeout_sec", 5.0)
        self.declare_parameter("lift_timeout_reverse_min_position", 0.01)
        self.declare_parameter("lift_publish_count", 10)
        self.declare_parameter("lift_publish_period_sec", 0.1)
        self.declare_parameter("lift_trigger_topic_suffix", "lift_cmd")
        self.declare_parameter(
            "target_visual_cmd_file",
            "/home/nsl/IsaacSim-ros_workspaces/humble_ws/src/capstone/amr_navigation/config/target_visual.json",
        )
        self.declare_parameter(
            "robot_log_dir",
            "/home/nsl/Capston_workspace",
        )
        self.declare_parameter(
            "result_dir",
            "/home/nsl/IsaacSim-ros_workspaces/result",
        )

        self.target_pallet = str(self.get_parameter("target_pallet").value)
        self.prim_structure_file = str(self.get_parameter("prim_structure_file").value)
        self.robot_structure_file = str(self.get_parameter("robot_structure_file").value)
        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self.pre_dock_distance = float(self.get_parameter("pre_dock_distance").value)
        self.pre_dock_position_handoff_tolerance = float(
            self.get_parameter("pre_dock_position_handoff_tolerance").value
        )
        self.amr2_pre_dock_position_handoff_tolerance = float(
            self.get_parameter("amr2_pre_dock_position_handoff_tolerance").value
        )
        self.axis_tolerance = float(self.get_parameter("axis_tolerance").value)
        self.amcl_ready_min_messages = int(
            self.get_parameter("amcl_ready_min_messages").value
        )
        self.amcl_ready_settle_sec = float(
            self.get_parameter("amcl_ready_settle_sec").value
        )
        self.dock_distance = float(self.get_parameter("dock_distance").value)
        self.dock_speed = float(self.get_parameter("dock_speed").value)
        self.dock_ramp_duration_sec = float(
            self.get_parameter("dock_ramp_duration_sec").value
        )
        self.dock_heading_gain = float(
            self.get_parameter("dock_heading_gain").value
        )
        self.dock_heading_stop_tolerance = float(
            self.get_parameter("dock_heading_stop_tolerance").value
        )
        self.dock_initial_heading_skip_tolerance = float(
            self.get_parameter("dock_initial_heading_skip_tolerance").value
        )
        self.stay_align_stop_tolerance = float(
            self.get_parameter("stay_align_stop_tolerance").value
        )
        self.stay_align_heading_gain = float(
            self.get_parameter("stay_align_heading_gain").value
        )
        self.stay_align_max_angular_speed = float(
            self.get_parameter("stay_align_max_angular_speed").value
        )
        self.stay_align_brake_tolerance = float(
            self.get_parameter("stay_align_brake_tolerance").value
        )
        self.stay_align_min_angular_speed = float(
            self.get_parameter("stay_align_min_angular_speed").value
        )
        self.loaded_stay_align_rotation_speed_scale = float(
            self.get_parameter("loaded_stay_align_rotation_speed_scale").value
        )
        self.loaded_stay_align_min_angular_speed = float(
            self.get_parameter("loaded_stay_align_min_angular_speed").value
        )
        self.dock_unloaded_align_heading_gain = float(
            self.get_parameter("dock_unloaded_align_heading_gain").value
        )
        self.dock_unloaded_align_max_angular_speed = float(
            self.get_parameter("dock_unloaded_align_max_angular_speed").value
        )
        self.dock_align_timeout_sec = float(
            self.get_parameter("dock_align_timeout_sec").value
        )
        self.dock_start_settle_sec = float(
            self.get_parameter("dock_start_settle_sec").value
        )
        self.loaded_dock_start_settle_sec = float(
            self.get_parameter("loaded_dock_start_settle_sec").value
        )
        self.pre_turn_forward_distance = float(
            self.get_parameter("pre_turn_forward_distance").value
        )
        self.pre_turn_forward_speed = float(
            self.get_parameter("pre_turn_forward_speed").value
        )
        self.pre_turn_heading_stop_tolerance = float(
            self.get_parameter("pre_turn_heading_stop_tolerance").value
        )
        self.pre_turn_heading_gain = float(
            self.get_parameter("pre_turn_heading_gain").value
        )
        self.pre_turn_max_angular_speed = float(
            self.get_parameter("pre_turn_max_angular_speed").value
        )
        self.pre_turn_brake_tolerance = float(
            self.get_parameter("pre_turn_brake_tolerance").value
        )
        self.pre_turn_min_angular_speed = float(
            self.get_parameter("pre_turn_min_angular_speed").value
        )
        self.loaded_pre_turn_rotation_speed_scale = float(
            self.get_parameter("loaded_pre_turn_rotation_speed_scale").value
        )
        self.loaded_pre_turn_min_angular_speed = float(
            self.get_parameter("loaded_pre_turn_min_angular_speed").value
        )
        self.angular_brake_start_tolerance = float(
            self.get_parameter("angular_brake_start_tolerance").value
        )
        self.angular_brake_exponent = float(
            self.get_parameter("angular_brake_exponent").value
        )
        self.angular_brake_min_speed = float(
            self.get_parameter("angular_brake_min_speed").value
        )
        self.angular_brake_final_tolerance = float(
            self.get_parameter("angular_brake_final_tolerance").value
        )
        self.angular_brake_final_exponent = float(
            self.get_parameter("angular_brake_final_exponent").value
        )
        self.angular_brake_final_min_speed = float(
            self.get_parameter("angular_brake_final_min_speed").value
        )
        self.dock_cross_track_heading_limit = float(
            self.get_parameter("dock_cross_track_heading_limit").value
        )
        self.dock_on_move_heading_gain = float(
            self.get_parameter("dock_on_move_heading_gain").value
        )
        self.dock_on_move_max_angular_speed = float(
            self.get_parameter("dock_on_move_max_angular_speed").value
        )
        self.dock_axis_follow_heading_limit = float(
            self.get_parameter("dock_axis_follow_heading_limit").value
        )
        self.dock_axis_follow_heading_gain = float(
            self.get_parameter("dock_axis_follow_heading_gain").value
        )
        self.dock_axis_follow_max_angular_speed = float(
            self.get_parameter("dock_axis_follow_max_angular_speed").value
        )
        self.dock_near_heading_gain = float(
            self.get_parameter("dock_near_heading_gain").value
        )
        self.dock_near_max_angular_speed = float(
            self.get_parameter("dock_near_max_angular_speed").value
        )
        self.dock_near_cross_track_heading_limit = float(
            self.get_parameter("dock_near_cross_track_heading_limit").value
        )
        self.dock_min_linear_ratio = float(
            self.get_parameter("dock_min_linear_ratio").value
        )
        self.dock_creep_distance = float(
            self.get_parameter("dock_creep_distance").value
        )
        self.dock_creep_speed = float(
            self.get_parameter("dock_creep_speed").value
        )
        self.dock_heading_drive_tolerance = float(
            self.get_parameter("dock_heading_drive_tolerance").value
        )
        self.dock_heading_correction_tolerance = float(
            self.get_parameter("dock_heading_correction_tolerance").value
        )
        self.dock_cross_track_gain = float(
            self.get_parameter("dock_cross_track_gain").value
        )
        self.dock_cross_track_heading_gain = float(
            self.get_parameter("dock_cross_track_heading_gain").value
        )
        self.dock_cross_track_tolerance = float(
            self.get_parameter("dock_cross_track_tolerance").value
        )
        self.dock_cross_track_stop_tolerance = float(
            self.get_parameter("dock_cross_track_stop_tolerance").value
        )
        self.dock_line_acquire_distance = float(
            self.get_parameter("dock_line_acquire_distance").value
        )
        self.dock_axis_lock_distance = float(
            self.get_parameter("dock_axis_lock_distance").value
        )
        self.dock_max_overshoot = float(
            self.get_parameter("dock_max_overshoot").value
        )
        self.dock_max_angular_speed = float(
            self.get_parameter("dock_max_angular_speed").value
        )
        self.dock_entry_offset = float(
            self.get_parameter("dock_entry_offset").value
        )
        self.blocker_reach_tolerance = float(
            self.get_parameter("blocker_reach_tolerance").value
        )
        self.pre_lift_stop_duration_sec = float(
            self.get_parameter("pre_lift_stop_duration_sec").value
        )
        self.lift_align_tolerance = float(
            self.get_parameter("lift_align_tolerance").value
        )
        self.lift_align_completion_tolerance = float(
            self.get_parameter("lift_align_completion_tolerance").value
        )
        self.lift_align_deadband = float(
            self.get_parameter("lift_align_deadband").value
        )
        self.lift_align_heading_gain = float(
            self.get_parameter("lift_align_heading_gain").value
        )
        self.lift_align_heading_gain_max = float(
            self.get_parameter("lift_align_heading_gain_max").value
        )
        self.lift_align_heading_gain_ramp_error = float(
            self.get_parameter("lift_align_heading_gain_ramp_error").value
        )
        self.lift_align_max_angular_speed = float(
            self.get_parameter("lift_align_max_angular_speed").value
        )
        self.lift_align_min_angular_speed = float(
            self.get_parameter("lift_align_min_angular_speed").value
        )
        self.lift_align_loaded_min_angular_speed = float(
            self.get_parameter("lift_align_loaded_min_angular_speed").value
        )
        self.lift_settle_sec = float(self.get_parameter("lift_settle_sec").value)
        self.lift_precheck_position_tolerance = float(
            self.get_parameter("lift_precheck_position_tolerance").value
        )
        self.lift_precheck_yaw_tolerance = float(
            self.get_parameter("lift_precheck_yaw_tolerance").value
        )
        self.reverse_arrive_tolerance = float(
            self.get_parameter("reverse_arrive_tolerance").value
        )
        self.reverse_speed = float(self.get_parameter("reverse_speed").value)
        self.reverse_ramp_duration_sec = float(
            self.get_parameter("reverse_ramp_duration_sec").value
        )
        self.reverse_ramp_start_speed = float(
            self.get_parameter("reverse_ramp_start_speed").value
        )
        self.reverse_brake_distance = float(
            self.get_parameter("reverse_brake_distance").value
        )
        self.reverse_brake_exponent = max(
            1.0, float(self.get_parameter("reverse_brake_exponent").value)
        )
        self.reverse_creep_speed = float(
            self.get_parameter("reverse_creep_speed").value
        )
        self.manual_terminal_brake_distance = float(
            self.get_parameter("manual_terminal_brake_distance").value
        )
        self.manual_terminal_creep_speed = float(
            self.get_parameter("manual_terminal_creep_speed").value
        )
        self.manual_terminal_final_brake_distance = float(
            self.get_parameter("manual_terminal_final_brake_distance").value
        )
        self.manual_terminal_final_speed = float(
            self.get_parameter("manual_terminal_final_speed").value
        )
        self.amr2_packing_backoff_distance = float(
            self.get_parameter("amr2_packing_backoff_distance").value
        )
        self.pre_dock_wait_radius = float(
            self.get_parameter("pre_dock_wait_radius").value
        )
        self.amr1_retreat_distance = float(
            self.get_parameter("amr1_retreat_distance").value
        )
        self.amr1_clear_distance_for_amr2 = float(
            self.get_parameter("amr1_clear_distance_for_amr2").value
        )
        self.amr2_clear_distance_for_amr1_replace = float(
            self.get_parameter("amr2_clear_distance_for_amr1_replace").value
        )
        self.amr2_packing_nav_min_travel_for_amr1_replace = float(
            self.get_parameter("amr2_packing_nav_min_travel_for_amr1_replace").value
        )
        self.amr1_replace_safety_stop_distance = float(
            self.get_parameter("amr1_replace_safety_stop_distance").value
        )
        self.amr1_replace_safety_resume_distance = float(
            self.get_parameter("amr1_replace_safety_resume_distance").value
        )
        self.amr2_start_delay_sec = float(
            self.get_parameter("amr2_start_delay_sec").value
        )
        self.amr2_wait_distance_before_pre_dock = float(
            self.get_parameter("amr2_wait_distance_before_pre_dock").value
        )
        self.amr2_wait_trigger_distance = float(
            self.get_parameter("amr2_wait_trigger_distance").value
        )
        self.amr2_wait_precancel_margin = float(
            self.get_parameter("amr2_wait_precancel_margin").value
        )
        self.amr2_wait_manual_approach_speed = float(
            self.get_parameter("amr2_wait_manual_approach_speed").value
        )
        self.amr2_wait_manual_approach_heading_gain = float(
            self.get_parameter("amr2_wait_manual_approach_heading_gain").value
        )
        self.amr2_wait_manual_approach_max_angular_speed = float(
            self.get_parameter("amr2_wait_manual_approach_max_angular_speed").value
        )
        self.amr2_wait_manual_approach_heading_tolerance = float(
            self.get_parameter("amr2_wait_manual_approach_heading_tolerance").value
        )
        self.stay_zone_arrive_tolerance = float(
            self.get_parameter("stay_zone_arrive_tolerance").value
        )
        self.stay_reverse_brake_distance = float(
            self.get_parameter("stay_reverse_brake_distance").value
        )
        self.stay_reverse_creep_speed = float(
            self.get_parameter("stay_reverse_creep_speed").value
        )
        self.packing_station_prim = str(
            self.get_parameter("packing_station_prim").value
        )
        self.packing_station_x = float(
            self.get_parameter("packing_station_x").value
        )
        self.packing_station_y = float(
            self.get_parameter("packing_station_y").value
        )
        self.packing_station_yaw = float(
            self.get_parameter("packing_station_yaw").value
        )
        self.packing_align_tolerance = float(
            self.get_parameter("packing_align_tolerance").value
        )
        self.packing_align_heading_gain = float(
            self.get_parameter("packing_align_heading_gain").value
        )
        self.packing_align_max_angular_speed = float(
            self.get_parameter("packing_align_max_angular_speed").value
        )
        self.loaded_motion_scale = float(
            self.get_parameter("loaded_motion_scale").value
        )
        self.loaded_rotation_speed_scale = float(
            self.get_parameter("loaded_rotation_speed_scale").value
        )
        self.lift_joint_name = str(self.get_parameter("lift_joint_name").value)
        self.lift_target_position = float(
            self.get_parameter("lift_target_position").value
        )
        self.lift_cmd_file = str(self.get_parameter("lift_cmd_file").value)
        self.lift_state_file = str(self.get_parameter("lift_state_file").value)
        self.lift_state_tolerance = float(
            self.get_parameter("lift_state_tolerance").value
        )
        self.lift_completion_tolerance = float(
            self.get_parameter("lift_completion_tolerance").value
        )
        self.lift_lower_completion_tolerance = float(
            self.get_parameter("lift_lower_completion_tolerance").value
        )
        self.lift_verify_timeout_sec = float(
            self.get_parameter("lift_verify_timeout_sec").value
        )
        self.lift_timeout_reverse_min_position = float(
            self.get_parameter("lift_timeout_reverse_min_position").value
        )
        self.lift_publish_count = int(self.get_parameter("lift_publish_count").value)
        self.lift_publish_period_sec = float(
            self.get_parameter("lift_publish_period_sec").value
        )
        self.lift_trigger_topic_suffix = str(
            self.get_parameter("lift_trigger_topic_suffix").value
        )
        self.target_visual_cmd_file = str(
            self.get_parameter("target_visual_cmd_file").value
        )
        self.robot_log_dir = str(self.get_parameter("robot_log_dir").value)
        self.result_dir = str(self.get_parameter("result_dir").value)
        self.pallets = load_top_level_pallets(self.prim_structure_file)
        self.robots = load_robot_specs(self.robot_structure_file)
        self._nav_clients: dict[str, ActionClient] = {}
        self._active_nav_context_by_namespace: dict[str, dict] = {}
        self._manual_control_namespaces: set[str] = set()
        self._cmd_vel_publishers: dict[str, object] = {}
        self._lift_trigger_publishers: dict[str, object] = {}
        self._pending_dispatch = None
        self._planning_started = False
        self._amcl_message_counts: dict[str, int] = {}
        self._amcl_first_msg_times: dict[str, object] = {}
        self._latest_amcl_pose_by_namespace: dict[str, tuple[float, float, float]] = {}
        self._amcl_subscriptions = []
        self._docking_state = None
        self._mission_state = None
        self._robot_log_paths = {
            namespace: os.path.join(self.robot_log_dir, f"{namespace}_log")
            for namespace in NAV2_NAMESPACE_BY_ROBOT.values()
        }
        self._result_log_path = os.path.join(self.result_dir, "result_log")
        self._docking_timer = self.create_timer(0.05, self._docking_step)
        self._coordination_timer = self.create_timer(0.2, self._coordination_step)

        self._reset_robot_log_files()
        self._reset_result_log_files()
        self._reset_lift_command_files()
        self._reset_target_visual_command_file()

        for namespace in NAV2_NAMESPACE_BY_ROBOT.values():
            subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{namespace}/amcl_pose",
                lambda msg, ns=namespace: self._handle_amcl_pose(msg, ns),
                10,
            )
            self._amcl_subscriptions.append(subscription)

        self.get_logger().info("Capstone task planner node started.")
        self._log_robot_inventory()
        self.get_logger().info(
            "Waiting for AMCL localization before starting the blocker planner."
        )
        self._amcl_wait_timer = self.create_timer(1.0, self._try_start_planning)
        self._dispatch_timer = self.create_timer(2.0, self._dispatch_pending_goal)

    def _log_robot_inventory(self) -> None:
        for robot in self.robots.values():
            self.get_logger().info(
                "Found AMR %s at (%.3f, %.3f, %.3f)"
                % (
                    robot.prim_path,
                    robot.pose.x,
                    robot.pose.y,
                    robot.pose.z,
                )
            )

    def _handle_amcl_pose(
        self, msg: PoseWithCovarianceStamped, namespace: str
    ) -> None:
        yaw = self._yaw_from_quaternion(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self._latest_amcl_pose_by_namespace[namespace] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        count = self._amcl_message_counts.get(namespace, 0) + 1
        self._amcl_message_counts[namespace] = count
        if namespace not in self._amcl_first_msg_times:
            self._amcl_first_msg_times[namespace] = self.get_clock().now()
            self.get_logger().info(
                "Received first /%s/amcl_pose at (%.3f, %.3f)"
                % (
                    namespace,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                )
            )
        elif count == self.amcl_ready_min_messages:
            self.get_logger().info(
                "/%s/amcl_pose reached %d messages."
                % (namespace, self.amcl_ready_min_messages)
            )

    def _get_robot_mission_state(self, namespace: str | None) -> str | None:
        if namespace is None or self._mission_state is None:
            return None
        robot_states = self._mission_state.get("robot_states")
        if not isinstance(robot_states, dict):
            return None
        return robot_states.get(namespace)

    def _set_robot_mission_state(
        self,
        namespace: str | None,
        state: str,
        reason: str,
    ) -> None:
        if namespace is None or self._mission_state is None:
            return
        robot_states = self._mission_state.setdefault("robot_states", {})
        previous_state = robot_states.get(namespace)
        if previous_state == state:
            return
        robot_states[namespace] = state
        self._log_robot_info(
            namespace,
            "Mission state changed: %s -> %s (%s)"
            % (
                "UNSET" if previous_state is None else previous_state,
                state,
                reason,
            ),
        )

    def _current_ros_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _build_result_metrics(self) -> dict:
        return {
            "amr1_move_start_time": None,
            "amr1_first_pre_dock_arrival_time": None,
            "amr1_digging_start_time": None,
            "amr2_returned_pre_dock_with_target_time": None,
            "amr1_move_to_relocation_start_time": None,
            "amr1_relocation_placed_time": None,
            "amr1_home_arrival_time": None,
            "amr1_nav2_total_sec": 0.0,
            "amr2_nav2_total_sec": 0.0,
            "amr1_nav2_segment_start": None,
            "amr2_nav2_segment_start": None,
            "move_duration_logged": False,
            "digging_duration_logged": False,
            "relocate_duration_logged": False,
            "total_duration_logged": False,
            "report_completed": False,
        }

    def _start_nav2_segment(self, namespace: str | None) -> None:
        if self._mission_state is None or namespace is None:
            return
        metrics = self._mission_state.setdefault(
            "result_metrics", self._build_result_metrics()
        )
        amr1_namespace = self._mission_state.get("amr1_namespace")
        amr2_namespace = self._mission_state.get("amr2_namespace")
        key = None
        if namespace == amr1_namespace:
            key = "amr1_nav2_segment_start"
        elif namespace == amr2_namespace:
            key = "amr2_nav2_segment_start"
        if key is None:
            return
        if metrics.get(key) is None:
            metrics[key] = self._current_ros_time_sec()

    def _end_nav2_segment(self, namespace: str | None) -> None:
        if self._mission_state is None or namespace is None:
            return
        metrics = self._mission_state.setdefault(
            "result_metrics", self._build_result_metrics()
        )
        amr1_namespace = self._mission_state.get("amr1_namespace")
        amr2_namespace = self._mission_state.get("amr2_namespace")
        if namespace == amr1_namespace:
            seg_key = "amr1_nav2_segment_start"
            total_key = "amr1_nav2_total_sec"
        elif namespace == amr2_namespace:
            seg_key = "amr2_nav2_segment_start"
            total_key = "amr2_nav2_total_sec"
        else:
            return
        seg_start = metrics.get(seg_key)
        if seg_start is None:
            return
        elapsed = max(0.0, self._current_ros_time_sec() - float(seg_start))
        metrics[total_key] = float(metrics.get(total_key, 0.0)) + elapsed
        metrics[seg_key] = None

    def _record_result_event(self, key: str, description: str) -> None:
        if self._mission_state is None:
            return
        metrics = self._mission_state.setdefault(
            "result_metrics",
            self._build_result_metrics(),
        )
        if metrics.get(key) is not None:
            return
        event_time = self._current_ros_time_sec()
        metrics[key] = event_time
        self.get_logger().info(
            "Result metric recorded: %s at %.3f (%s)"
            % (key, event_time, description)
        )
        self._append_result_log(
            "INFO",
            "Result metric recorded: %s at %.3f (%s)"
            % (key, event_time, description),
        )
        self._write_result_report()

    def _format_result_duration(self, start: float | None, end: float | None) -> str:
        if start is None or end is None:
            return "계산 대기 중"
        return f"{max(0.0, end - start):.3f} s"

    def _append_result_duration_logs_if_ready(self, metrics: dict) -> None:
        move_start = metrics.get("amr1_move_start_time")
        digging_start = metrics.get("amr1_digging_start_time")
        amr2_return = metrics.get("amr2_returned_pre_dock_with_target_time")
        move_to_relocation_start = metrics.get("amr1_move_to_relocation_start_time")
        relocation_placed = metrics.get("amr1_relocation_placed_time")
        home_arrival = metrics.get("amr1_home_arrival_time")

        if (
            not metrics.get("digging_duration_logged", False)
            and digging_start is not None
            and amr2_return is not None
        ):
            self._append_result_log(
                "INFO",
                "DIGGING 시간 확정: %.3f s (AMR1 DIGGING 시작 -> AMR2 target pallet lift 후 shared pre_dock 도착)"
                % max(0.0, amr2_return - digging_start),
            )
            metrics["digging_duration_logged"] = True

        if (
            not metrics.get("relocate_duration_logged", False)
            and move_to_relocation_start is not None
            and relocation_placed is not None
        ):
            self._append_result_log(
                "INFO",
                "RELOCATE 시간 확정: %.3f s (AMR1 MOVE_TO_RELOCATION 시작 -> AMR1 LOWERING 완료)"
                % max(0.0, relocation_placed - move_to_relocation_start),
            )
            metrics["relocate_duration_logged"] = True

        if (
            not metrics.get("total_duration_logged", False)
            and move_start is not None
            and home_arrival is not None
        ):
            self._append_result_log(
                "INFO",
                "전체 시간 확정: %.3f s (주문 시작 -> AMR1 초기 위치 도착)"
                % max(0.0, home_arrival - move_start),
            )
            metrics["total_duration_logged"] = True

    def _write_result_report(self) -> None:
        if self._mission_state is None:
            return

        metrics = self._mission_state.setdefault(
            "result_metrics",
            self._build_result_metrics(),
        )
        move_start = metrics.get("amr1_move_start_time")
        first_pre_dock = metrics.get("amr1_first_pre_dock_arrival_time")
        digging_start = metrics.get("amr1_digging_start_time")
        amr2_return = metrics.get("amr2_returned_pre_dock_with_target_time")
        move_to_relocation_start = metrics.get("amr1_move_to_relocation_start_time")
        relocation_placed = metrics.get("amr1_relocation_placed_time")
        home_arrival = metrics.get("amr1_home_arrival_time")
        amr1_nav2_total = float(metrics.get("amr1_nav2_total_sec", 0.0) or 0.0)
        amr2_nav2_total = float(metrics.get("amr2_nav2_total_sec", 0.0) or 0.0)

        digging_duration = self._format_result_duration(digging_start, amr2_return)
        relocate_duration = self._format_result_duration(
            move_to_relocation_start, relocation_placed
        )
        total_duration = self._format_result_duration(move_start, home_arrival)

        status = "completed" if home_arrival is not None else "running"
        if move_start is None:
            status = "initialized"

        event_labels = {
            "amr1_move_start_time": "AMR1 MOVE 시작 (주문 시작)",
            "amr1_first_pre_dock_arrival_time": "AMR1 첫 shared pre_dock 도착",
            "amr1_digging_start_time": "AMR1 DIGGING 시작",
            "amr2_returned_pre_dock_with_target_time": "AMR2 target pallet lift 후 shared pre_dock 도착",
            "amr1_move_to_relocation_start_time": "AMR1 MOVE_TO_RELOCATION 시작",
            "amr1_relocation_placed_time": "AMR1 blocker pallet 재배치 완료 (LOWERING 완료)",
            "amr1_home_arrival_time": "AMR1 초기 위치 도착",
        }

        markdown_lines = [
            "# Result",
            "",
            f"- 상태: `{status}`",
            f"- 마지막 갱신 ROS time: `{self._current_ros_time_sec():.3f}`",
            "",
            "## 결과",
            f"- DIGGING 시간: `{digging_duration}`",
            "  AMR1이 DIGGING을 시작한 순간부터 AMR2가 target pallet을 lift하고 shared pre_dock에 도착할 때까지",
            f"- RELOCATE 시간: `{relocate_duration}`",
            "  AMR1이 MOVE_TO_RELOCATION을 시작한 순간부터 LOWERING을 완료할 때까지",
            f"- 전체 시간: `{total_duration}`",
            "  주문을 받고 움직이기 시작한 순간부터 AMR1이 초기 위치에 도착할 때까지",
            f"- AMR1 Nav2 이동 시간: `{amr1_nav2_total:.3f} s`",
            "  AMR1이 Nav2로 이동한 총 시간 (모든 leg 합계)",
            f"- AMR2 Nav2 이동 시간: `{amr2_nav2_total:.3f} s`",
            "  AMR2가 Nav2로 이동한 총 시간 (모든 leg 합계)",
            "",
            "## 이벤트 시각",
        ]
        for key, label in event_labels.items():
            value = metrics.get(key)
            markdown_lines.append(
                f"- {label}: `{value:.3f}`" if value is not None else f"- {label}: `대기 중`"
            )

        payload = {
            "status": status,
            "updated_ros_time_sec": round(self._current_ros_time_sec(), 3),
            "durations": {
                "digging_time_sec": None if digging_start is None or amr2_return is None else round(amr2_return - digging_start, 3),
                "relocate_time_sec": None if move_to_relocation_start is None or relocation_placed is None else round(relocation_placed - move_to_relocation_start, 3),
                "total_time_sec": None if move_start is None or home_arrival is None else round(home_arrival - move_start, 3),
                "amr1_nav2_total_sec": round(amr1_nav2_total, 3),
                "amr2_nav2_total_sec": round(amr2_nav2_total, 3),
            },
            "events": {
                key: (None if metrics.get(key) is None else round(metrics.get(key), 3))
                for key in event_labels
            },
        }

        try:
            self._append_result_duration_logs_if_ready(metrics)
            os.makedirs(self.result_dir, exist_ok=True)
            markdown_path = os.path.join(self.result_dir, "latest_result.md")
            json_path = os.path.join(self.result_dir, "latest_result.json")
            with open(markdown_path, "w", encoding="utf-8") as handle:
                handle.write("\n".join(markdown_lines) + "\n")
            with open(json_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, ensure_ascii=False)
            if status == "completed" and not metrics.get("report_completed", False):
                self._append_result_log(
                    "INFO",
                    "Mission result completed: digging_time=%s relocate_time=%s total_time=%s amr1_nav2=%.3fs amr2_nav2=%.3fs"
                    % (
                        digging_duration,
                        relocate_duration,
                        total_duration,
                        amr1_nav2_total,
                        amr2_nav2_total,
                    ),
                )
                metrics["report_completed"] = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Failed to write result report in %s: %s"
                % (self.result_dir, exc)
            )

    def _is_initial_amr1_blocker_docking(
        self,
        namespace: str,
        blocker,
        post_reverse_stage: str | None,
    ) -> bool:
        if self._mission_state is None:
            return False
        if namespace != self._mission_state.get("amr1_namespace"):
            return False
        if post_reverse_stage is not None:
            return False
        mission_blocker = self._mission_state.get("blocker")
        return mission_blocker is not None and blocker.name == mission_blocker.name

    def _try_start_planning(self) -> None:
        if self._planning_started:
            return

        required_namespaces = {
            namespace
            for robot_name, namespace in NAV2_NAMESPACE_BY_ROBOT.items()
            if robot_name in self.robots
        }

        if not required_namespaces:
            self.get_logger().warning(
                "No AMR robots were loaded, so planner startup is skipped."
            )
            self._planning_started = True
            return

        ready_namespaces, waiting_details = self._get_ready_and_waiting_namespaces(
            required_namespaces
        )

        if len(ready_namespaces) != len(required_namespaces):
            self.get_logger().info(
                "Still waiting for AMCL readiness: %s"
                % ", ".join(waiting_details)
            )
            return

        self._planning_started = True
        if self._amcl_wait_timer is not None:
            self._amcl_wait_timer.cancel()
            self._amcl_wait_timer = None
        self.get_logger().info(
            "AMCL localization is ready for all active robots. Starting planner."
        )
        self._plan_blocker_assignment()

    def _get_ready_and_waiting_namespaces(self, required_namespaces):
        ready_namespaces = []
        waiting_details = []
        now = self.get_clock().now()
        for namespace in sorted(required_namespaces):
            msg_count = self._amcl_message_counts.get(namespace, 0)
            if msg_count < self.amcl_ready_min_messages:
                waiting_details.append(
                    "%s(msgs=%d/%d)"
                    % (namespace, msg_count, self.amcl_ready_min_messages)
                )
                continue

            first_msg_time = self._amcl_first_msg_times.get(namespace)
            if first_msg_time is None:
                waiting_details.append("%s(first_msg=missing)" % namespace)
                continue

            settled_sec = (now - first_msg_time).nanoseconds / 1e9
            if settled_sec < self.amcl_ready_settle_sec:
                waiting_details.append(
                    "%s(settle=%.1f/%.1fs)"
                    % (namespace, settled_sec, self.amcl_ready_settle_sec)
                )
                continue

            ready_namespaces.append(namespace)

        return ready_namespaces, waiting_details

    def _plan_blocker_assignment(self) -> None:
        if self.target_pallet not in self.pallets:
            self.get_logger().warning(
                "Target pallet %s was not found in %s"
                % (self.target_pallet, self.prim_structure_file)
            )
            return

        blocker_name = blocker_name_from_target(self.target_pallet)
        if blocker_name is None:
            self.get_logger().warning(
                "Target pallet %s does not follow the *_02 naming rule"
                % self.target_pallet
            )
            return

        blocker = self.pallets.get(blocker_name)
        if blocker is None:
            self.get_logger().warning(
                "Blocker pallet %s was not found in %s"
                % (blocker_name, self.prim_structure_file)
            )
            return

        target = self.pallets[self.target_pallet]
        pre_dock_process = pre_dock_process_from_pair(
            blocker=blocker,
            target=target,
            pre_dock_distance=self.pre_dock_distance,
            axis_tolerance=self.axis_tolerance,
        )
        if pre_dock_process is None:
            self.get_logger().warning(
                "Failed to compute pre-dock process for blocker %s and target %s"
                % (blocker.name, target.name)
            )
            return

        if not self.robots:
            self.get_logger().warning(
                "No AMR prims were found in %s" % self.robot_structure_file
            )
            return

        amr1_robot, distance = select_closest_robot(self.robots, blocker.pose)
        amr1_namespace = NAV2_NAMESPACE_BY_ROBOT.get(amr1_robot.name)
        if not amr1_namespace:
            self.get_logger().warning(
                "No Nav2 namespace mapping was found for robot %s" % amr1_robot.name
            )
            return

        remaining_robots = {
            name: robot
            for name, robot in self.robots.items()
            if name != amr1_robot.name
        }
        if not remaining_robots:
            self.get_logger().warning(
                "Only one AMR is available. AMR2 coordination is skipped."
            )

        target_pre_dock_process = PreDockProcessResult(
            blocker_name=target.name,
            target_name=target.name,
            approach_axis=pre_dock_process.approach_axis,
            approach_sign=pre_dock_process.approach_sign,
            # AMR2 uses the same blocker-based pre_dock as AMR1.
            # Both robots first gather 2 m in front of the blocker pallet.
            pre_dock_pose=pre_dock_process.pre_dock_pose,
            pre_dock_yaw=pre_dock_process.pre_dock_yaw,
        )
        amr2_robot = None
        amr2_namespace = None
        if remaining_robots:
            amr2_robot, _ = select_closest_robot(
                remaining_robots, pre_dock_process.pre_dock_pose
            )
            amr2_namespace = NAV2_NAMESPACE_BY_ROBOT.get(amr2_robot.name)
            if not amr2_namespace:
                self.get_logger().warning(
                    "No Nav2 namespace mapping was found for AMR2 robot %s" % amr2_robot.name
                )
                amr2_robot = None
        self._mission_state = {
            "blocker": blocker,
            "target": target,
            "amr1_robot": amr1_robot,
            "amr1_namespace": amr1_namespace,
            "amr2_robot": amr2_robot,
            "amr2_namespace": amr2_namespace,
            "blocker_pre_dock_process": pre_dock_process,
            "target_pre_dock_process": target_pre_dock_process,
            "amr2_started": False,
            "amr2_waiting": False,
            "amr2_wait_canceled": False,
            "amr2_wait_gate_triggered": False,
            "amr2_wait_precanceled": False,
            "amr2_at_pre_dock": False,
            "amr2_packing_sent": False,
            "amr1_retreat_complete": False,
            "amr1_departed_pre_dock_with_load": False,
            "amr2_docking_started": False,
            "amr1_retreat_sent": False,
            "amr2_goal_context": None,
            "amr1_goal_context": None,
            "amr1_goal_sent_time": None,
            "amr1_replace_started": False,
            "amr1_replace_waiting_for_clearance": False,
            "amr1_replace_completed": False,
            "relocate": False,
            "amr1_home_sent": False,
            "robot_states": {},
            "result_metrics": self._build_result_metrics(),
        }

        self.get_logger().info("Target pallet: %s" % self.target_pallet)
        self._write_target_visual_command(target.prim_path)
        self.get_logger().info(
            "Target pallet highlight requested for %s" % target.prim_path
        )
        self.get_logger().info("Blocker pallet: %s" % blocker.name)
        self.get_logger().info("Blocker prim path: %s" % blocker.prim_path)
        self.get_logger().info(
            "Blocker position: (%.3f, %.3f, %.3f)"
            % (blocker.pose.x, blocker.pose.y, blocker.pose.z)
        )
        self.get_logger().info(
            "Assigned robot: %s (distance %.3f m)"
            % (amr1_robot.prim_path, distance)
        )
        if amr2_robot is not None:
            self.get_logger().info(
                "Assigned AMR2: %s -> blocker pre-dock (%.3f, %.3f), hold when first entering %.1f m radius"
                % (
                    amr2_robot.prim_path,
                    target_pre_dock_process.pre_dock_pose.x,
                    target_pre_dock_process.pre_dock_pose.y,
                    self.amr2_wait_trigger_distance,
                )
            )
        self.get_logger().info(
            "Pre-dock process: axis=%s sign=%+d pre_dock=(%.3f, %.3f, %.3f) yaw=%.3f"
            % (
                pre_dock_process.approach_axis,
                pre_dock_process.approach_sign,
                pre_dock_process.pre_dock_pose.x,
                pre_dock_process.pre_dock_pose.y,
                pre_dock_process.pre_dock_pose.z,
                pre_dock_process.pre_dock_yaw,
            )
        )
        self.get_logger().info(
            "Next step: send a Nav2 goal to namespace %s for %s"
            % (amr1_namespace, amr1_robot.prim_path)
        )
        self._set_robot_mission_state(
            amr1_namespace,
            "MOVE_TO_PRE_DOCK",
            "AMR1 is assigned to the shared blocker pre_dock and will move there.",
        )
        if amr2_namespace is not None:
            self._set_robot_mission_state(
                amr2_namespace,
                "WAIT_DELAYED_START",
                "AMR2 is waiting for its delayed start toward the shared blocker pre_dock.",
            )
        self._pending_dispatch = {
            "robot": amr1_robot,
            "namespace": amr1_namespace,
            "blocker": blocker,
            "target": target,
            "pre_dock_process": pre_dock_process,
            "stage": "amr1_pre_dock",
        }
        self._write_result_report()

    def _reset_target_visual_command_file(self) -> None:
        self._write_target_visual_command("")

    def _write_target_visual_command(self, prim_path: str) -> None:
        payload = {
            "target_prim_path": prim_path,
            "display_color_rgb": [1.0, 1.0, 0.0],
        }
        try:
            target_dir = os.path.dirname(self.target_visual_cmd_file)
            if target_dir:
                os.makedirs(target_dir, exist_ok=True)
            with open(self.target_visual_cmd_file, "w", encoding="utf-8") as handle:
                json.dump(payload, handle)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Failed to write target visual command file %s: %s"
                % (self.target_visual_cmd_file, exc)
            )

    def _dispatch_pending_goal(self) -> None:
        if self._pending_dispatch is None:
            return

        namespace = self._pending_dispatch["namespace"]
        robot = self._pending_dispatch["robot"]
        pre_dock_process = self._pending_dispatch["pre_dock_process"]

        action_name = f"/{namespace}/navigate_to_pose"
        client = self._nav_clients.get(namespace)
        if client is None:
            client = ActionClient(self, NavigateToPose, action_name)
            self._nav_clients[namespace] = client

        if not client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                "Waiting for Nav2 action server: %s" % action_name
            )
            return

        self._send_nav_goal(self._pending_dispatch)
        self._pending_dispatch = None

    def _send_nav_goal(self, context) -> None:
        namespace = context["namespace"]
        robot = context["robot"]
        pre_dock_process = context["pre_dock_process"]
        stage = context.get("stage", "pre_dock")
        action_name = f"/{namespace}/navigate_to_pose"
        client = self._nav_clients.get(namespace)
        if client is None:
            client = ActionClient(self, NavigateToPose, action_name)
            self._nav_clients[namespace] = client

        if not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warning(
                "Nav2 action server is not available for %s" % action_name
            )
            return

        nav_goal_yaw = pre_dock_process.pre_dock_yaw
        if stage.endswith("pre_dock"):
            current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
            if current_pose is not None:
                nav_goal_yaw = current_pose[2]
        self.get_logger().info(
            "Sending %s Nav2 goal to %s for robot %s -> (%.3f, %.3f), yaw=%.3f%s"
            % (
                stage,
                action_name,
                robot.prim_path,
                pre_dock_process.pre_dock_pose.x,
                pre_dock_process.pre_dock_pose.y,
                nav_goal_yaw,
                " (pre_dock yaw alignment disabled)" if stage.endswith("pre_dock") else "",
            )
        )
        stage_to_state = {
            "amr1_pre_dock": "MOVE_TO_PRE_DOCK",
            "amr2_pre_dock": "MOVE_TO_PRE_DOCK",
            "amr2_packing_station": "MOVING_TO_PACKING",
            "amr1_return_home": "MOVING_HOME",
            "amr1_replace_pre_dock": "MOVE_TO_RELOCATION",
        }
        if stage in stage_to_state:
            self._set_robot_mission_state(
                namespace,
                stage_to_state[stage],
                "Nav2 is actively moving this AMR during %s." % stage,
            )
        if (
            self._mission_state is not None
            and stage == "amr1_pre_dock"
            and self._mission_state.get("amr1_goal_sent_time") is None
        ):
            self._mission_state["amr1_goal_sent_time"] = self.get_clock().now()
        if stage == "amr1_pre_dock":
            self._record_result_event(
                "amr1_move_start_time",
                "AMR1 Nav2 goal toward the shared pre_dock was sent.",
            )
        if stage == "amr2_packing_station":
            current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
            if current_pose is not None:
                context["nav_start_pose"] = current_pose
        if self._mission_state is not None and stage == "amr1_pre_dock":
            self._mission_state["amr1_goal_context"] = context
        send_future = client.send_goal_async(
            self._create_nav_goal(
                pre_dock_process.pre_dock_pose.x,
                pre_dock_process.pre_dock_pose.y,
                nav_goal_yaw,
            )
        )
        send_future.add_done_callback(
            lambda future, ctx=context: self._handle_goal_response(
                future, ctx
            )
        )

    def _handle_goal_response(self, future, context) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Pre-dock Nav2 goal was rejected.")
            return

        self.get_logger().info(
            "%s Nav2 goal accepted."
            % context.get("stage", "pre_dock")
        )
        stage = context.get("stage", "pre_dock")
        context["goal_handle"] = goal_handle
        self._active_nav_context_by_namespace[context["namespace"]] = context
        self._start_nav2_segment(context.get("namespace"))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future, ctx=context: self._handle_nav_result(
                result_future, ctx
            )
        )

    def _handle_nav_result(self, future, context) -> None:
        result = future.result()
        stage = context.get("stage", "pre_dock")
        namespace = context["namespace"]
        active_context = self._active_nav_context_by_namespace.get(namespace)
        if active_context is context:
            self._active_nav_context_by_namespace.pop(namespace, None)
        self._end_nav2_segment(namespace)
        if context.get("planner_manual_handoff") and result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                "%s Nav2 goal result arrived after planner switched %s to manual docking. Ignoring canceled result."
                % (stage, namespace)
            )
            return
        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            and stage == "amr2_pre_dock"
            and self._mission_state is not None
            and self._mission_state.get("amr2_wait_canceled")
        ):
            self._mission_state["amr2_wait_canceled"] = False
            self.get_logger().info(
                "AMR2 pre-dock goal was intentionally canceled while AMR2 was holding at its distance gate before pre-dock."
            )
            return

        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            and stage == "amr1_replace_pre_dock"
            and context.get("planner_safety_stop")
        ):
            self.get_logger().info(
                "AMR1 replace pre-dock goal was intentionally canceled for safety clearance."
            )
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                "%s Nav2 goal failed with status %d"
                % (stage, result.status)
            )
            return

        robot = context["robot"]
        blocker = context["blocker"]
        pre_dock_process = context["pre_dock_process"]
        if stage == "amr1_pre_dock":
            current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
            if current_pose is None:
                self.get_logger().warning(
                    "No AMCL pose is available for %s, docking phase is skipped."
                    % namespace
                )
                return
            self._maybe_start_amr2()
            self._start_docking_from_current_pose(
                namespace=namespace,
                robot=robot,
                blocker=blocker,
                pre_dock_process=pre_dock_process,
                current_pose=current_pose,
                reason="AMR1 pre-dock goal succeeded",
            )
            return

        if stage == "amr2_pre_dock":
            if self._mission_state is not None:
                self._mission_state["amr2_at_pre_dock"] = True
                self._mission_state["amr2_waiting"] = False
            if self._docking_state is None:
                current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
                if current_pose is None:
                    self.get_logger().warning(
                        "No AMCL pose is available for %s, AMR2 docking is skipped."
                        % namespace
                    )
                    return
                self.get_logger().info(
                    "AMR2 pre-dock reached. Starting target docking."
                )
                self._start_amr2_target_docking(current_pose)
            else:
                self._set_robot_mission_state(
                    namespace,
                    "WAIT_FOR_SLOT",
                    "AMR2 reached the shared pre_dock and is waiting there for the manual docking slot to become free.",
                )
                self.get_logger().info(
                    "AMR2 reached target pre-dock and is waiting there for the manual docking slot to become free."
                )
            return

        if stage == "amr1_stay_zone":
            self.get_logger().info(
                "AMR1 stay_zone Nav2 goal succeeded. AMR2 may resume target docking flow."
            )
            if self._mission_state is not None:
                self._mission_state["amr1_retreat_complete"] = True
            self._maybe_resume_amr2()
            return

        if stage == "amr2_packing_station":
            self.get_logger().info(
                "AMR2 reached PackingStation. Mission completed."
            )
            self._set_robot_mission_state(
                namespace,
                "COMPLETED",
                "AMR2 reached PackingStation and finished its mission.",
            )
            return

        if stage == "amr1_replace_pre_dock":
            current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
            blocker_pre_dock_process = None if self._mission_state is None else self._mission_state.get("blocker_pre_dock_process")
            target = None if self._mission_state is None else self._mission_state.get("target")
            if current_pose is None or blocker_pre_dock_process is None or target is None:
                self.get_logger().warning(
                    "No AMCL pose or target process is available for %s, AMR1 replacement docking is skipped."
                    % namespace
                )
                return
            self._start_docking_from_current_pose(
                namespace=namespace,
                robot=robot,
                blocker=target,
                pre_dock_process=blocker_pre_dock_process,
                current_pose=current_pose,
                reason="AMR1 reached the shared blocker pre_dock for blocker re-placement",
                lift_target_position=0.0,
                initial_carrying_load=True,
                post_reverse_stage="amr1_return_home",
            )
            return

        if stage == "amr1_return_home":
            self.get_logger().info(
                "AMR1 returned to its initial position. Re-placement flow completed."
            )
            if self._mission_state is not None:
                self._mission_state["amr1_replace_completed"] = True
            self._record_result_event(
                "amr1_home_arrival_time",
                "AMR1 returned to its initial position.",
            )
            self._set_robot_mission_state(
                namespace,
                "COMPLETED",
                "AMR1 returned to its initial position and finished the re-placement mission.",
            )
            return

        if stage == "amr1_retreat":
            self.get_logger().info(
                "AMR1 retreat goal succeeded. AMR1 has cleared the pre-dock area."
            )
            if self._mission_state is not None:
                self._mission_state["amr1_retreat_complete"] = True
            self._maybe_resume_amr2()
            return

        if stage == "blocker_entry":
            current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
            if current_pose is None:
                self.get_logger().warning(
                    "No AMCL pose is available for %s, reverse phase is skipped."
                    % namespace
                )
                return
            self.get_logger().info(
                "Blocker-entry goal succeeded for %s. Starting lift phase."
                % robot.prim_path
            )
            self._publish_stop(namespace)
            self._docking_state = {
                "phase": "lift",
                "namespace": namespace,
                "robot": robot,
                "blocker": blocker,
                "approach_axis": pre_dock_process.approach_axis,
                "approach_sign": pre_dock_process.approach_sign,
                "start_pose": current_pose,
                "reverse_heading": pre_dock_process.pre_dock_yaw,
                "lift_publish_remaining": self.lift_publish_count,
                "last_lift_publish_time": None,
            }
            return

    def _coordination_step(self) -> None:
        if self._mission_state is None:
            return

        self._log_pre_dock_tracking_once_per_second()
        self._maybe_start_amr2()

        amr2_namespace = self._mission_state.get("amr2_namespace")
        if not amr2_namespace:
            return

        if self._mission_state.get("amr2_waiting"):
            self._publish_stop(amr2_namespace)

        amr1_namespace = self._mission_state.get("amr1_namespace")
        if not amr1_namespace:
            return

        amr1_pose = self._latest_amcl_pose_by_namespace.get(amr1_namespace)
        amr2_pose = self._latest_amcl_pose_by_namespace.get(amr2_namespace)
        blocker_pre_dock_process = self._mission_state.get("blocker_pre_dock_process")
        target_pre_dock_process = self._mission_state.get("target_pre_dock_process")
        amr1_goal_context = self._mission_state.get("amr1_goal_context")
        amr2_goal_context = self._mission_state.get("amr2_goal_context")
        if (
            amr1_pose is None
            or amr2_pose is None
            or blocker_pre_dock_process is None
            or target_pre_dock_process is None
        ):
            return

        blocker_pre_dock_pose = blocker_pre_dock_process.pre_dock_pose
        target_pre_dock_pose = target_pre_dock_process.pre_dock_pose
        amr1_dist = math.hypot(
            amr1_pose[0] - blocker_pre_dock_pose.x,
            amr1_pose[1] - blocker_pre_dock_pose.y,
        )
        amr2_dist = math.hypot(
            amr2_pose[0] - target_pre_dock_pose.x,
            amr2_pose[1] - target_pre_dock_pose.y,
        )
        amr_robot_separation = math.hypot(
            amr1_pose[0] - amr2_pose[0],
            amr1_pose[1] - amr2_pose[1],
        )
        amr1_state = self._get_robot_mission_state(amr1_namespace)
        amr2_state = self._get_robot_mission_state(amr2_namespace)

        if (
            self._docking_state is None
            and amr1_dist <= self.pre_dock_position_handoff_tolerance
            and not self._mission_state.get("relocate")
            and amr1_goal_context is not None
            and amr1_goal_context.get("stage") == "amr1_pre_dock"
        ):
            amr1_robot = self._mission_state.get("amr1_robot")
            blocker = self._mission_state.get("blocker")
            if amr1_robot is not None and blocker is not None:
                goal_handle = amr1_goal_context.get("goal_handle")
                if goal_handle is not None:
                    goal_handle.cancel_goal_async()
                    amr1_goal_context["planner_manual_handoff"] = True
                self._publish_stop(amr1_namespace)
                self.get_logger().info(
                    "AMR1 reached pre-dock position tolerance (distance=%.3f m <= %.3f m). Stopping at pre-dock and switching from Nav2 pre-dock to manual docking."
                    % (
                        amr1_dist,
                        self.pre_dock_position_handoff_tolerance,
                    )
                )
                self._maybe_start_amr2()
                self._start_docking_from_current_pose(
                    namespace=amr1_namespace,
                    robot=amr1_robot,
                    blocker=blocker,
                    pre_dock_process=blocker_pre_dock_process,
                    current_pose=amr1_pose,
                    reason="AMR1 pre-dock position tolerance reached",
                )
                return

        if (
            self._docking_state is None
            and self._mission_state.get("amr2_at_pre_dock")
            and not self._mission_state.get("amr2_docking_started")
        ):
            self.get_logger().info(
                "AMR2 is already holding at pre-dock. Starting manual target docking now that the manual docking slot is free."
            )
            self._start_amr2_target_docking(amr2_pose)
            return

        if (
            self._mission_state.get("amr2_waiting")
            and not self._mission_state.get("amr2_at_pre_dock")
            and not self._mission_state.get("amr2_docking_started")
            and amr1_state in ("ALIGNING_TO_STAY", "MOVING_TO_STAY", "WAIT")
            and amr2_state == "WAIT_AT_GATE"
            and amr1_dist > self.amr1_clear_distance_for_amr2
        ):
            amr2_robot = self._mission_state.get("amr2_robot")
            target = self._mission_state.get("target")
            target_pre_dock_process = self._mission_state.get("target_pre_dock_process")
            if (
                amr2_robot is not None
                and target is not None
                and target_pre_dock_process is not None
            ):
                context = {
                    "stage": "amr2_pre_dock",
                    "robot": amr2_robot,
                    "namespace": amr2_namespace,
                    "blocker": target,
                    "target": target,
                    "pre_dock_process": target_pre_dock_process,
                }
                self._mission_state["amr2_waiting"] = False
                self._mission_state["amr2_wait_precanceled"] = False
                self._mission_state["amr2_goal_context"] = context
                self.get_logger().info(
                    "AMR1 is in WAIT state and moved %.2f m away from the shared pre_dock. Releasing AMR2 from WAIT and sending it back to the shared pre_dock."
                    % amr1_dist
                )
                self._send_nav_goal(context)
                return

        if (
            not self._mission_state.get("amr2_wait_gate_triggered")
            and not self._mission_state.get("amr2_waiting")
            and not self._mission_state.get("amr2_at_pre_dock")
            and not self._mission_state.get("amr2_docking_started")
            and amr2_goal_context is not None
            and amr2_goal_context.get("stage") == "amr2_pre_dock"
        ):
            precancel_distance = (
                self.amr2_wait_trigger_distance + self.amr2_wait_precancel_margin
            )
            if (
                not self._mission_state.get("amr2_wait_precanceled")
                and amr2_dist <= precancel_distance
                and amr2_dist > self.amr2_wait_trigger_distance
            ):
                goal_handle = amr2_goal_context.get("goal_handle")
                if goal_handle is not None:
                    goal_handle.cancel_goal_async()
                self._mission_state["amr2_wait_precanceled"] = True
                self._mission_state["amr2_wait_canceled"] = True
                self._set_robot_mission_state(
                    amr2_namespace,
                    "APPROACHING_GATE",
                    "AMR2 canceled Nav2 in the pre-cancel margin and is now manually creeping toward the wait gate.",
                )
                self.get_logger().info(
                    "AMR2 entered the pre-cancel margin for the %.1f m wait gate (distance=%.3f m <= %.3f m). Canceling Nav2 early and switching to slow manual approach so the final stop lands near %.1f m."
                    % (
                        self.amr2_wait_trigger_distance,
                        amr2_dist,
                        precancel_distance,
                        self.amr2_wait_trigger_distance,
                    )
                )
                return

            if (
                self._mission_state.get("amr2_wait_precanceled")
                and amr2_dist > self.amr2_wait_trigger_distance
            ):
                self._approach_amr2_wait_gate_manually(
                    namespace=amr2_namespace,
                    current_pose=amr2_pose,
                    pre_dock_pose=target_pre_dock_pose,
                    distance_to_pre_dock=amr2_dist,
                )
                return

            if amr2_dist <= self.amr2_wait_trigger_distance:
                self._publish_stop(amr2_namespace)
                self._mission_state["amr2_wait_gate_triggered"] = True
                self._mission_state["amr2_waiting"] = True
                self._mission_state["amr2_wait_canceled"] = True
                self._mission_state["amr2_wait_precanceled"] = False
                self._set_robot_mission_state(
                    amr2_namespace,
                    "WAIT_AT_GATE",
                    "AMR2 reached its pre_dock wait gate and is holding there until AMR1 clears the shared pre_dock.",
                )
                self.get_logger().info(
                    "AMR2 reached the %.1f m wait gate (distance=%.3f m). Stopping immediately and holding there until AMR1 departs pre-dock with the lifted pallet."
                    % (
                        self.amr2_wait_trigger_distance,
                        amr2_dist,
                    )
                )
                return

        if (
            not self._mission_state.get("amr2_docking_started")
            and amr2_dist <= self.amr2_pre_dock_position_handoff_tolerance
        ):
            amr2_robot = self._mission_state.get("amr2_robot")
            target = self._mission_state.get("target")
            if amr2_robot is not None and target is not None:
                if amr2_goal_context is not None:
                    goal_handle = amr2_goal_context.get("goal_handle")
                    if goal_handle is not None:
                        goal_handle.cancel_goal_async()
                self._mission_state["amr2_at_pre_dock"] = True
                self._mission_state["amr2_waiting"] = False
                self._mission_state["amr2_wait_canceled"] = True
                self._mission_state["amr2_wait_precanceled"] = False
                if self._docking_state is None:
                    if amr2_goal_context is not None:
                        amr2_goal_context["planner_manual_handoff"] = True
                    self._publish_stop(amr2_namespace)
                    self.get_logger().info(
                        "AMR2 reached pre-dock position tolerance (distance=%.3f m <= %.3f m). Stopping at pre-dock and switching from Nav2 pre-dock to manual docking."
                        % (
                            amr2_dist,
                            self.amr2_pre_dock_position_handoff_tolerance,
                        )
                    )
                    self._start_amr2_target_docking(amr2_pose)
                else:
                    self._publish_stop(amr2_namespace)
                    self._set_robot_mission_state(
                        amr2_namespace,
                        "WAIT_FOR_SLOT",
                        "AMR2 is holding at the shared pre_dock until the manual docking slot becomes free.",
                    )
                    self.get_logger().info(
                        "AMR2 reached pre-dock position tolerance (distance=%.3f m <= %.3f m). Nav2 pre-dock is canceled and AMR2 is holding at pre-dock until the manual docking slot becomes free."
                        % (
                            amr2_dist,
                            self.amr2_pre_dock_position_handoff_tolerance,
                        )
                    )
                return

        if amr2_goal_context is None:
            return

        if (
            self._docking_state is None
            and not self._mission_state.get("amr1_replace_started")
            and amr2_namespace is not None
        ):
            active_amr2_context = self._active_nav_context_by_namespace.get(amr2_namespace)
            if (
                active_amr2_context is not None
                and active_amr2_context.get("stage") == "amr2_packing_station"
                and amr1_state == "WAIT"
                and amr2_state == "MOVING_TO_PACKING"
            ):
                if amr2_dist >= self.amr2_clear_distance_for_amr1_replace:
                    self.get_logger().info(
                        "AMR2 is in MOVE state toward PackingStation and is %.2f m away from the shared pre_dock. AMR1 may leave WAIT and start RELOCATION now."
                        % (
                            amr2_dist,
                        )
                    )
                    self._maybe_start_amr1_replace_flow()

        active_amr1_context = self._active_nav_context_by_namespace.get(amr1_namespace)
        active_amr2_context = self._active_nav_context_by_namespace.get(amr2_namespace)
        if (
            active_amr1_context is not None
            and active_amr1_context.get("stage") == "amr1_replace_pre_dock"
            and active_amr2_context is not None
            and active_amr2_context.get("stage") == "amr2_packing_station"
            and amr_robot_separation <= self.amr1_replace_safety_stop_distance
        ):
            goal_handle = active_amr1_context.get("goal_handle")
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
                active_amr1_context["planner_safety_stop"] = True
            self._publish_stop(amr1_namespace)
            self._mission_state["amr1_replace_waiting_for_clearance"] = True
            self.get_logger().warning(
                "AMR1 replace pre-dock Nav2 goal paused for safety: robot separation %.2f m <= %.2f m while AMR2 is leaving toward PackingStation."
                % (
                    amr_robot_separation,
                    self.amr1_replace_safety_stop_distance,
                )
            )
            return

        if (
            self._mission_state.get("amr1_replace_waiting_for_clearance")
            and active_amr2_context is not None
            and active_amr2_context.get("stage") == "amr2_packing_station"
            and amr_robot_separation >= self.amr1_replace_safety_resume_distance
        ):
            self._maybe_resume_amr1_replace_flow()

    def _maybe_start_amr2(self) -> None:
        if self._mission_state is None:
            return
        if self._mission_state.get("amr2_started"):
            return
        amr1_goal_sent_time = self._mission_state.get("amr1_goal_sent_time")
        if amr1_goal_sent_time is None:
            return
        elapsed = (self.get_clock().now() - amr1_goal_sent_time).nanoseconds / 1e9
        if elapsed < self.amr2_start_delay_sec:
            return

        amr2_robot = self._mission_state.get("amr2_robot")
        amr2_namespace = self._mission_state.get("amr2_namespace")
        target_pre_dock_process = self._mission_state.get("target_pre_dock_process")
        if amr2_robot is None or amr2_namespace is None or target_pre_dock_process is None:
            self.get_logger().info("AMR2 is not available. Continuing with AMR1 only.")
            return

        context = {
            "stage": "amr2_pre_dock",
            "robot": amr2_robot,
            "namespace": amr2_namespace,
            "blocker": self._mission_state["target"],
            "target": self._mission_state["target"],
            "pre_dock_process": target_pre_dock_process,
        }
        self._mission_state["amr2_started"] = True
        self._mission_state["amr2_goal_context"] = context
        self._set_robot_mission_state(
            amr2_namespace,
            "MOVE_TO_PRE_DOCK",
            "AMR2 finished its delayed wait and is moving toward the shared pre_dock.",
        )
        self.get_logger().info(
            "AMR1 started. After %.1fs delay, starting AMR2 toward blocker pre-dock. AMR2 will stop at the first point where its distance to pre-dock becomes %.1f m or less."
            % (self.amr2_start_delay_sec, self.amr2_wait_trigger_distance)
        )
        self._send_nav_goal(context)

    def _maybe_resume_amr2(self) -> None:
        if self._mission_state is None:
            return
        if self._mission_state.get("amr2_docking_started"):
            return

        if self._mission_state.get("amr2_at_pre_dock") and self._docking_state is None:
            amr2_namespace = self._mission_state.get("amr2_namespace")
            current_pose = None if amr2_namespace is None else self._latest_amcl_pose_by_namespace.get(amr2_namespace)
            if current_pose is not None:
                self.get_logger().info(
                    "AMR2 is already at pre-dock and the manual docking slot is free. Starting target docking."
                )
                self._start_amr2_target_docking(current_pose)
            return

        if self._mission_state.get("amr2_waiting"):
            amr2_robot = self._mission_state.get("amr2_robot")
            amr2_namespace = self._mission_state.get("amr2_namespace")
            target_pre_dock_process = self._mission_state.get("target_pre_dock_process")
            if amr2_robot is not None and amr2_namespace is not None and target_pre_dock_process is not None:
                context = {
                    "stage": "amr2_pre_dock",
                    "robot": amr2_robot,
                    "namespace": amr2_namespace,
                    "blocker": self._mission_state["target"],
                    "target": self._mission_state["target"],
                    "pre_dock_process": target_pre_dock_process,
                }
                self.get_logger().info(
                    "AMR1 is clear of blocker pre-dock. Resuming AMR2 from its hold point toward target pre-dock."
                )
                self._mission_state["amr2_waiting"] = False
                self._mission_state["amr2_goal_context"] = context
                self._send_nav_goal(context)

    def _start_amr2_target_docking(self, current_pose: tuple[float, float, float]) -> None:
        if self._mission_state is None or self._mission_state.get("amr2_docking_started"):
            return
        amr2_namespace = self._mission_state.get("amr2_namespace")
        amr2_robot = self._mission_state.get("amr2_robot")
        target = self._mission_state.get("target")
        target_pre_dock_process = self._mission_state.get("target_pre_dock_process")
        if amr2_namespace is None or amr2_robot is None or target is None or target_pre_dock_process is None:
            return
        self._mission_state["amr2_docking_started"] = True
        self._set_robot_mission_state(
            amr2_namespace,
            "DIGGING",
            "AMR2 reached the shared pre_dock and is starting its rotate-forward-lift digging flow.",
        )
        self._log_robot_info(
            amr2_namespace,
            "AMR2 target docking flow: pre_dock -> pre_turn_forward -> dock -> lift_align -> lift -> reverse"
        )
        self._start_docking_from_current_pose(
            namespace=amr2_namespace,
            robot=amr2_robot,
            blocker=target,
            pre_dock_process=target_pre_dock_process,
            current_pose=current_pose,
            reason="AMR2 target pre-dock goal succeeded",
        )

    def _compute_target_pre_dock_pose(
        self,
        pallet,
        approach_axis: str,
        docking_sign: int,
        distance: float,
    ) -> Pose3D:
        if approach_axis == "x":
            return Pose3D(
                pallet.pose.x - (docking_sign * distance),
                pallet.pose.y,
                pallet.pose.z,
            )
        return Pose3D(
            pallet.pose.x,
            pallet.pose.y - (docking_sign * distance),
            pallet.pose.z,
        )

    def _compute_wait_pose_before_pre_dock(
        self,
        pre_dock_pose: Pose3D,
        approach_axis: str,
        reference_robot_pose: Pose3D | None,
        distance_before_pre_dock: float,
    ) -> Pose3D:
        if approach_axis == "x":
            wait_sign = -1 if reference_robot_pose is not None and reference_robot_pose.y < pre_dock_pose.y else 1
            return Pose3D(
                pre_dock_pose.x,
                pre_dock_pose.y + (wait_sign * distance_before_pre_dock),
                pre_dock_pose.z,
            )
        wait_sign = -1 if reference_robot_pose is not None and reference_robot_pose.x < pre_dock_pose.x else 1
        return Pose3D(
            pre_dock_pose.x + (wait_sign * distance_before_pre_dock),
                pre_dock_pose.y,
                pre_dock_pose.z,
            )

    def _compute_amr1_retreat_pose(self, amr1_pose: tuple[float, float, float]) -> tuple[Pose3D, float]:
        mission = self._mission_state
        pre_dock_pose = mission["blocker_pre_dock_process"].pre_dock_pose
        approach_axis = mission["blocker_pre_dock_process"].approach_axis
        amr2_robot = mission.get("amr2_robot")
        amr2_reference_pose = None if amr2_robot is None else amr2_robot.pose

        if approach_axis == "x":
            amr2_side = -1 if amr2_reference_pose is not None and amr2_reference_pose.y < pre_dock_pose.y else 1
            retreat_sign = -amr2_side
            retreat_pose = Pose3D(
                pre_dock_pose.x,
                pre_dock_pose.y + (retreat_sign * self.amr1_retreat_distance),
                pre_dock_pose.z,
            )
        else:
            amr2_side = -1 if amr2_reference_pose is not None and amr2_reference_pose.x < pre_dock_pose.x else 1
            retreat_sign = -amr2_side
            retreat_pose = Pose3D(
                pre_dock_pose.x + (retreat_sign * self.amr1_retreat_distance),
                pre_dock_pose.y,
                pre_dock_pose.z,
            )

        if approach_axis == "x":
            stay_heading = self._axis_heading("y", -retreat_sign)
        else:
            stay_heading = self._axis_heading("x", -retreat_sign)

        return retreat_pose, stay_heading

    def _compute_packing_station_pose(self) -> Pose3D:
        prim_pose = load_named_prim_pose(
            self.prim_structure_file,
            self.packing_station_prim,
        )
        if prim_pose is not None:
            return prim_pose

        if self.packing_station_prim.endswith("PackingStation"):
            legacy_prim_pose = load_named_prim_pose(
                self.prim_structure_file,
                self.packing_station_prim.replace("PackingStation", "PackingStaion"),
            )
            if legacy_prim_pose is not None:
                return legacy_prim_pose

        self.get_logger().warning(
            "PackingStation prim %s was not found in %s. Falling back to configured coordinates."
            % (self.packing_station_prim, self.prim_structure_file)
        )
        return Pose3D(self.packing_station_x, self.packing_station_y, 0.0)

    def _build_amr1_replace_context(self):
        if self._mission_state is None:
            return None
        amr1_robot = self._mission_state.get("amr1_robot")
        amr1_namespace = self._mission_state.get("amr1_namespace")
        target = self._mission_state.get("target")
        blocker_pre_dock_process = self._mission_state.get("blocker_pre_dock_process")
        if (
            amr1_robot is None
            or amr1_namespace is None
            or target is None
            or blocker_pre_dock_process is None
        ):
            return None
        return {
            "stage": "amr1_replace_pre_dock",
            "robot": amr1_robot,
            "namespace": amr1_namespace,
            "blocker": target,
            "target": target,
            "pre_dock_process": blocker_pre_dock_process,
        }

    def _maybe_start_amr1_replace_flow(self) -> None:
        if self._mission_state is None or self._mission_state.get("amr1_replace_started"):
            return
        amr1_namespace = self._mission_state.get("amr1_namespace")
        current_pose = (
            None
            if amr1_namespace is None
            else self._latest_amcl_pose_by_namespace.get(amr1_namespace)
        )
        if current_pose is None:
            return
        self._mission_state["amr1_replace_started"] = True
        self._mission_state["amr1_replace_waiting_for_clearance"] = False
        self._record_result_event(
            "amr1_move_to_relocation_start_time",
            "AMR1 entered MOVE_TO_RELOCATION (manual drive toward shared pre_dock for blocker re-placement).",
        )
        self.get_logger().info(
            "AMR2 is safely clear. AMR1 will now manually drive back to the shared blocker pre_dock so the blocker pallet can be placed into the former target pallet slot."
        )
        self._start_move_to_relocation(current_pose)

    def _maybe_resume_amr1_replace_flow(self) -> None:
        if (
            self._mission_state is None
            or not self._mission_state.get("amr1_replace_started")
            or not self._mission_state.get("amr1_replace_waiting_for_clearance")
        ):
            return
        amr1_namespace = self._mission_state.get("amr1_namespace")
        current_pose = (
            None
            if amr1_namespace is None
            else self._latest_amcl_pose_by_namespace.get(amr1_namespace)
        )
        if current_pose is None:
            return
        self._mission_state["amr1_replace_waiting_for_clearance"] = False
        self.get_logger().info(
            "AMR2 is safely clear again. Resuming AMR1 manual drive toward the shared blocker pre_dock."
        )
        self._start_move_to_relocation(current_pose)

    def _start_move_to_relocation(self, current_pose: tuple[float, float, float]) -> None:
        if self._mission_state is None:
            return
        amr1_robot = self._mission_state.get("amr1_robot")
        amr1_namespace = self._mission_state.get("amr1_namespace")
        target = self._mission_state.get("target")
        blocker_pre_dock_process = self._mission_state.get("blocker_pre_dock_process")
        if (
            amr1_robot is None
            or amr1_namespace is None
            or target is None
            or blocker_pre_dock_process is None
        ):
            return

        active_context = self._active_nav_context_by_namespace.get(amr1_namespace)
        if active_context is not None:
            goal_handle = active_context.get("goal_handle")
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
                active_context["planner_manual_handoff"] = True

        self._manual_control_namespaces.add(amr1_namespace)

        pre_dock_pose = blocker_pre_dock_process.pre_dock_pose
        target_heading = math.atan2(
            pre_dock_pose.y - current_pose[1],
            pre_dock_pose.x - current_pose[0],
        )

        self._set_robot_mission_state(
            amr1_namespace,
            "MOVE_TO_RELOCATION",
            "AMR1 is manually driving from stay_zone back to the shared pre_dock with the blocker pallet still raised.",
        )
        self._log_robot_info(
            amr1_namespace,
            "Starting manual move_to_relocation: current=(%.3f, %.3f, %.3f) -> pre_dock=(%.3f, %.3f) target_heading=%.3f"
            % (
                current_pose[0],
                current_pose[1],
                current_pose[2],
                pre_dock_pose.x,
                pre_dock_pose.y,
                target_heading,
            ),
        )
        self._docking_state = {
            "phase": "move_to_relocation",
            "namespace": amr1_namespace,
            "robot": amr1_robot,
            "blocker": target,
            "approach_axis": blocker_pre_dock_process.approach_axis,
            "approach_sign": blocker_pre_dock_process.approach_sign,
            "start_pose": current_pose,
            "pre_dock_pose": pre_dock_pose,
            "target_heading": target_heading,
            "carrying_load": True,
            "post_reverse_stage": "amr1_return_home",
            "lift_target_position": 0.0,
        }

    def _move_to_relocation_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]

        if (
            self._mission_state is not None
            and self._mission_state.get("amr1_replace_waiting_for_clearance")
        ):
            self._publish_stop(namespace)
            return

        pre_dock_pose = self._docking_state["pre_dock_pose"]
        base_heading = self._docking_state["target_heading"]
        current_x, current_y, current_yaw = current_pose

        distance = math.hypot(pre_dock_pose.x - current_x, pre_dock_pose.y - current_y)

        if distance <= self.pre_dock_position_handoff_tolerance:
            self._publish_stop(namespace)
            self._log_robot_info(
                namespace,
                "Manual move_to_relocation reached pre_dock (distance=%.3f m <= %.3f m). Switching to manual docking for blocker re-placement."
                % (distance, self.pre_dock_position_handoff_tolerance),
            )
            mission = self._mission_state
            target = None if mission is None else mission.get("target")
            blocker_pre_dock_process = (
                None if mission is None else mission.get("blocker_pre_dock_process")
            )
            robot = self._docking_state.get("robot")
            if target is None or blocker_pre_dock_process is None or robot is None:
                self._manual_control_namespaces.discard(namespace)
                self._docking_state = None
                return
            self._start_docking_from_current_pose(
                namespace=namespace,
                robot=robot,
                blocker=target,
                pre_dock_process=blocker_pre_dock_process,
                current_pose=current_pose,
                reason="Manual move_to_relocation arrived at shared pre_dock",
                lift_target_position=0.0,
                initial_carrying_load=True,
                post_reverse_stage="amr1_return_home",
            )
            return

        direction_x = math.cos(base_heading)
        direction_y = math.sin(base_heading)
        along_remaining = (
            direction_x * (pre_dock_pose.x - current_x)
            + direction_y * (pre_dock_pose.y - current_y)
        )
        relative_x = current_x - pre_dock_pose.x
        relative_y = current_y - pre_dock_pose.y
        cross_track_error = (-direction_y * relative_x) + (direction_x * relative_y)
        heading_correction = math.atan(
            self.dock_cross_track_heading_gain * cross_track_error
        )
        near_target = distance <= self.dock_line_acquire_distance
        axis_lock_active = (
            distance <= self.dock_axis_lock_distance
            and abs(cross_track_error) <= self.dock_cross_track_tolerance
        )
        heading_limit = self.dock_cross_track_heading_limit
        angular_gain = self.dock_on_move_heading_gain
        angular_limit = self.dock_on_move_max_angular_speed
        if near_target:
            heading_limit = self.dock_near_cross_track_heading_limit
            angular_gain = self.dock_near_heading_gain
            angular_limit = self.dock_near_max_angular_speed
        if axis_lock_active:
            heading_correction = 0.0
        else:
            heading_correction = max(
                -heading_limit,
                min(heading_limit, heading_correction),
            )
        desired_heading = self._normalize_angle(base_heading - heading_correction)
        yaw_error = self._normalize_angle(desired_heading - current_yaw)

        if self._docking_state.get("move_to_relocation_start_time") is None:
            self._docking_state["move_to_relocation_start_time"] = self.get_clock().now()
        ramp_elapsed = (
            self.get_clock().now() - self._docking_state["move_to_relocation_start_time"]
        ).nanoseconds / 1e9
        ramp_progress = min(1.0, ramp_elapsed / max(self.dock_ramp_duration_sec, 1e-6))
        base_speed = self.dock_speed * self.loaded_motion_scale
        ramped_max_speed = self.dock_creep_speed + (
            base_speed - self.dock_creep_speed
        ) * ramp_progress

        linear_x = base_speed * (self.dock_min_linear_ratio if near_target else 1.0)
        linear_x = min(linear_x, ramped_max_speed)
        if distance <= self.dock_creep_distance:
            linear_x = min(linear_x, self.dock_creep_speed)
        linear_x = self._apply_terminal_linear_brake(linear_x, distance)

        max_angular = angular_limit * self.loaded_rotation_speed_scale
        angular_z = max(
            -max_angular,
            min(max_angular, angular_gain * yaw_error),
        )

        self._publish_cmd_vel(namespace, linear_x, angular_z)

    def _send_amr1_home_goal(self, current_pose: tuple[float, float, float]) -> None:
        if self._mission_state is None or self._mission_state.get("amr1_home_sent"):
            return
        amr1_robot = self._mission_state.get("amr1_robot")
        amr1_namespace = self._mission_state.get("amr1_namespace")
        if amr1_robot is None or amr1_namespace is None:
            return
        self._mission_state["amr1_home_sent"] = True
        current_yaw = current_pose[2]
        home_pose = Pose3D(amr1_robot.pose.x, amr1_robot.pose.y, amr1_robot.pose.z)
        context = {
            "stage": "amr1_return_home",
            "robot": amr1_robot,
            "namespace": amr1_namespace,
            "blocker": self._mission_state.get("blocker"),
            "target": self._mission_state.get("target"),
            "pre_dock_process": PreDockProcessResult(
                blocker_name=amr1_robot.name,
                target_name=amr1_robot.name,
                approach_axis="x",
                approach_sign=1,
                pre_dock_pose=home_pose,
                pre_dock_yaw=current_yaw,
            ),
        }
        self.get_logger().info(
            "AMR1 finished placing the blocker pallet into the former target pallet slot. Sending Nav2 goal back to its initial position at (%.3f, %.3f)."
            % (home_pose.x, home_pose.y)
        )
        self._send_nav_goal(context)

    def _docking_step(self) -> None:
        if self._docking_state is None:
            return

        namespace = self._docking_state["namespace"]
        current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
        if current_pose is None:
            return

        self._log_manual_phase_state_once_per_second(current_pose)
        phase = self._docking_state["phase"]
        if phase == "align":
            self._align_step(current_pose)
        elif phase == "pre_turn_forward":
            self._pre_turn_forward_step(current_pose)
        elif phase == "start_settle":
            self._start_settle_step()
        elif phase == "dock":
            self._dock_forward_step(current_pose)
        elif phase == "pre_lift_stop":
            self._pre_lift_stop_step(current_pose)
        elif phase == "lift_align":
            self._lift_align_step(current_pose)
        elif phase == "settle":
            self._settle_before_lift_step()
        elif phase == "lift":
            self._lift_step()
        elif phase == "reverse":
            self._reverse_step(current_pose)
        elif phase == "packing_backoff":
            self._packing_backoff_step(current_pose)
        elif phase == "stay_align":
            self._stay_align_step(current_pose)
        elif phase == "stay_reverse":
            self._stay_reverse_step(current_pose)
        elif phase == "move_to_relocation":
            self._move_to_relocation_step(current_pose)

    def _start_docking_from_current_pose(
        self,
        namespace: str,
        robot,
        blocker,
        pre_dock_process,
        current_pose: tuple[float, float, float],
        reason: str,
        lift_target_position: float | None = None,
        initial_carrying_load: bool = False,
        post_reverse_stage: str | None = None,
    ) -> None:
        if (
            self._mission_state is not None
            and namespace == self._mission_state.get("amr1_namespace")
            and self._mission_state.get("relocate")
            and post_reverse_stage != "amr1_return_home"
        ):
            target = self._mission_state.get("target")
            if target is not None:
                blocker = target
            lift_target_position = 0.0
            initial_carrying_load = True
            post_reverse_stage = "amr1_return_home"
            self._log_robot_info(
                namespace,
                "Override docking args because relocate=True (caller did not pass relocation stage). Forcing RELOCATING flow with lift_target=0.0 and post_reverse_stage=amr1_return_home.",
            )
        active_context = self._active_nav_context_by_namespace.get(namespace)
        if active_context is not None:
            goal_handle = active_context.get("goal_handle")
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
                active_context["planner_manual_handoff"] = True
                self.get_logger().info(
                    "Canceled active Nav2 goal for %s before switching to manual docking."
                    % namespace
                )
        self._manual_control_namespaces.add(namespace)
        docking_sign = -pre_dock_process.approach_sign
        axis_heading = self._axis_heading(
            pre_dock_process.approach_axis,
            docking_sign,
        )
        current_yaw = current_pose[2]
        yaw_error = self._normalize_angle(axis_heading - current_yaw)
        initial_phase = "pre_turn_forward"
        self._log_robot_info(
            namespace,
            "%s for %s. Starting docking phase (%s -> dock), axis_heading=%.3f current_yaw=%.3f error=%.3f blocker=(%.3f, %.3f) pre_dock=(%.3f, %.3f, %.3f)"
            % (
                reason,
                robot.prim_path,
                initial_phase,
                axis_heading,
                current_yaw,
                yaw_error,
                blocker.pose.x,
                blocker.pose.y,
                pre_dock_process.pre_dock_pose.x,
                pre_dock_process.pre_dock_pose.y,
                pre_dock_process.pre_dock_yaw,
            ),
        )
        self._log_robot_info(
            namespace,
            "Pre-turn forward start pose: pos=(%.3f, %.3f) yaw=%.3f"
            % (
                current_pose[0],
                current_pose[1],
                current_pose[2],
            ),
        )
        if post_reverse_stage == "amr1_return_home":
            self._set_robot_mission_state(
                namespace,
                "RELOCATING",
                "AMR1 is using the shared pre_dock before placing the blocker pallet into the former target pallet slot.",
            )
        else:
            self._set_robot_mission_state(
                namespace,
                "DIGGING",
                "This AMR reached the shared pre_dock and is starting its rotate-forward-lift digging flow.",
            )
        if self._is_initial_amr1_blocker_docking(
            namespace=namespace,
            blocker=blocker,
            post_reverse_stage=post_reverse_stage,
        ):
            self._record_result_event(
                "amr1_first_pre_dock_arrival_time",
                "AMR1 switched from Nav2 pre_dock travel to manual docking at the shared pre_dock.",
            )
            self._record_result_event(
                "amr1_digging_start_time",
                "AMR1 entered DIGGING at the shared pre_dock.",
            )
        self._docking_state = {
            "phase": initial_phase,
            "namespace": namespace,
            "robot": robot,
            "blocker": blocker,
            "approach_axis": pre_dock_process.approach_axis,
            "approach_sign": docking_sign,
            "start_pose": current_pose,
            "pre_dock_pose": pre_dock_process.pre_dock_pose,
            "align_start_pose": current_pose,
            "target_heading": axis_heading,
            "reverse_heading": axis_heading,
            "pre_turn_forward_start_pose": current_pose,
            "lift_publish_remaining": self.lift_publish_count,
            "last_lift_publish_time": None,
            "last_heading_log_time": None,
            "align_start_time": self.get_clock().now(),
            "start_settle_start_time": self.get_clock().now(),
            "lift_target_position": self.lift_target_position if lift_target_position is None else lift_target_position,
            "carrying_load": initial_carrying_load,
            "post_reverse_stage": post_reverse_stage,
        }

    def _pre_turn_forward_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        current_x, current_y, current_yaw = current_pose
        desired_heading = self._docking_state["target_heading"]
        yaw_error = self._normalize_angle(desired_heading - current_yaw)
        if abs(yaw_error) <= self.pre_turn_heading_stop_tolerance:
            self._publish_stop(namespace)
            if self.dock_start_settle_sec <= 0.0:
                self._log_robot_info(
                    namespace,
                    "Pre-dock heading alignment finished at current=(%.3f, %.3f, %.3f). Starting pallet-axis docking immediately."
                    % (
                        current_x,
                        current_y,
                        current_yaw,
                    ),
                )
                self._docking_state["phase"] = "dock"
                self._dock_forward_step(current_pose)
                return
            self._log_robot_info(
                namespace,
                "Pre-dock heading alignment finished at current=(%.3f, %.3f, %.3f). Holding cmd_vel=0 briefly before pallet-axis docking."
                % (
                    current_x,
                    current_y,
                    current_yaw,
                ),
            )
            self._docking_state["phase"] = "start_settle"
            self._docking_state["start_settle_start_time"] = self.get_clock().now()
            return

        self._run_forward_in_place_rotation_step(
            namespace=namespace,
            phase="pre_turn_forward",
            current_pose=current_pose,
            target_heading=desired_heading,
            stop_tolerance=self.pre_turn_brake_tolerance,
            gain=self.pre_turn_heading_gain,
            max_speed=self.pre_turn_max_angular_speed,
            reason="Rotate in place toward pallet axis before starting pallet-axis docking",
        )

    def _align_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        current_x, current_y, current_yaw = current_pose
        base_target_yaw = self._docking_state["target_heading"]
        desired_heading = base_target_yaw
        yaw_error = self._normalize_angle(desired_heading - current_yaw)
        align_start_time = self._docking_state.get("align_start_time")
        align_elapsed_sec = 0.0
        if align_start_time is not None:
            align_elapsed_sec = (
                self.get_clock().now() - align_start_time
            ).nanoseconds / 1e9
        align_start_pose = self._docking_state.get("align_start_pose", current_pose)
        drift = math.hypot(current_x - align_start_pose[0], current_y - align_start_pose[1])

        if abs(yaw_error) <= self.dock_initial_heading_skip_tolerance:
            self.get_logger().info(
                "Alignment phase finished at pos=(%.3f, %.3f) yaw=%.3f desired_heading=%.3f drift_from_pre_dock=%.3f. Starting forward docking motion immediately."
                % (
                    current_x,
                    current_y,
                    current_yaw,
                    desired_heading,
                    drift,
                )
            )
            self._docking_state["phase"] = "dock"
            self._docking_state["target_heading"] = base_target_yaw
            return

        if align_elapsed_sec >= self.dock_align_timeout_sec:
            self.get_logger().info(
                "Alignment timeout reached after %.2fs at pos=(%.3f, %.3f) yaw=%.3f desired_heading=%.3f with yaw_error=%.3f rad and drift_from_pre_dock=%.3f m. "
                "Starting forward docking motion immediately and correcting while moving."
                % (
                    align_elapsed_sec,
                    current_x,
                    current_y,
                    current_yaw,
                    desired_heading,
                    yaw_error,
                    drift,
                )
            )
            self._docking_state["phase"] = "dock"
            self._docking_state["target_heading"] = base_target_yaw
            return

        self._run_forward_in_place_rotation_step(
            namespace=namespace,
            phase="align",
            current_pose=current_pose,
            target_heading=desired_heading,
            stop_tolerance=self.dock_initial_heading_skip_tolerance,
            gain=self.dock_unloaded_align_heading_gain,
            max_speed=self.dock_unloaded_align_max_angular_speed,
            reason=(
                "Rotate in place toward pallet approach axis "
                f"(blocker=({self._docking_state['blocker'].pose.x:.3f}, "
                f"{self._docking_state['blocker'].pose.y:.3f}), "
                f"axis={self._docking_state['approach_axis']}, "
                f"sign={self._docking_state['approach_sign']:+d})"
            ),
        )

    def _start_settle_step(self) -> None:
        namespace = self._docking_state["namespace"]
        self._publish_stop(namespace)
        start_time = self._docking_state.get("start_settle_start_time")
        if start_time is None:
            self._docking_state["start_settle_start_time"] = self.get_clock().now()
            return

        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        carrying_load = bool(self._docking_state.get("carrying_load"))
        settle_duration = (
            self.loaded_dock_start_settle_sec if carrying_load else self.dock_start_settle_sec
        )
        if elapsed < settle_duration:
            return

        current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
        if current_pose is not None:
            self._log_robot_info(
                namespace,
                "Forward docking start pose: pos=(%.3f, %.3f) yaw=%.3f"
                % (
                    current_pose[0],
                    current_pose[1],
                    current_pose[2],
                ),
            )
        self._log_robot_info(
            namespace,
            "Pre-dock stop hold finished. Starting forward docking motion.",
        )
        self._docking_state["phase"] = "dock"
        if current_pose is not None:
            self._dock_forward_step(current_pose)

    def _dock_forward_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        blocker = self._docking_state["blocker"]
        approach_axis = self._docking_state["approach_axis"]
        approach_sign = self._docking_state["approach_sign"]
        current_x, current_y, current_yaw = current_pose

        direction_x = 0.0
        direction_y = 0.0
        if approach_axis == "x":
            direction_x = float(approach_sign)
        else:
            direction_y = float(approach_sign)

        relative_x = current_x - blocker.pose.x
        relative_y = current_y - blocker.pose.y
        cross_track_error = (-direction_y * relative_x) + (direction_x * relative_y)
        target_x = blocker.pose.x + (direction_x * self.dock_entry_offset)
        target_y = blocker.pose.y + (direction_y * self.dock_entry_offset)
        along_remaining = (
            direction_x * (target_x - current_x)
            + direction_y * (target_y - current_y)
        )
        target_distance = math.hypot(target_x - current_x, target_y - current_y)
        lift_target_position = self._docking_state.get(
            "lift_target_position",
            self.lift_target_position,
        )
        lift_action_label = (
            "lowering the lift for pallet placement"
            if lift_target_position <= self.lift_completion_tolerance
            else "raising the lift for pallet pickup"
        )

        if along_remaining <= self.blocker_reach_tolerance:
            self._publish_stop(namespace)
            self._log_robot_info(
                namespace,
                "Reached blocker pallet target: target=(%.3f, %.3f) distance=%.3f along=%.3f cross=%.3f. Holding wheels at zero for %.2fs before final alignment for %s."
                % (
                    target_x,
                    target_y,
                    target_distance,
                    along_remaining,
                    cross_track_error,
                    self.pre_lift_stop_duration_sec,
                    lift_action_label,
                ),
            )
            self._docking_state["phase"] = "pre_lift_stop"
            self._docking_state["dock_start_time"] = None
            self._docking_state["pre_lift_stop_start_time"] = self.get_clock().now()
            return

        base_heading = self._docking_state["target_heading"]
        axis_heading = base_heading
        heading_correction = math.atan(
            self.dock_cross_track_heading_gain * cross_track_error
        )
        axis_error = self._normalize_angle(axis_heading - current_yaw)

        near_blocker = along_remaining <= self.dock_line_acquire_distance
        axis_lock_active = (
            along_remaining <= self.dock_axis_lock_distance
            and abs(cross_track_error) <= self.dock_cross_track_tolerance
        )
        if along_remaining < -self.dock_max_overshoot:
            self.get_logger().warning(
                "Docking overshoot exceeded %.3f m (along=%.3f, cross=%.3f). Stopping docking."
                % (
                    self.dock_max_overshoot,
                    along_remaining,
                    cross_track_error,
                )
            )
            self._publish_stop(namespace)
            self._docking_state = None
            return

        if self._docking_state.get("dock_start_time") is None:
            self._docking_state["dock_start_time"] = self.get_clock().now()
        ramp_elapsed = (
            self.get_clock().now() - self._docking_state["dock_start_time"]
        ).nanoseconds / 1e9
        ramp_progress = min(1.0, ramp_elapsed / max(self.dock_ramp_duration_sec, 1e-6))
        ramped_max_speed = self.dock_creep_speed + (self.dock_speed - self.dock_creep_speed) * ramp_progress

        linear_x = self.dock_speed * (self.dock_min_linear_ratio if near_blocker else 1.0)
        linear_x = min(linear_x, ramped_max_speed)
        if along_remaining <= self.dock_creep_distance:
            linear_x = min(linear_x, self.dock_creep_speed)
        heading_limit = self.dock_cross_track_heading_limit
        angular_gain = self.dock_on_move_heading_gain
        angular_limit = self.dock_on_move_max_angular_speed
        if along_remaining <= self.dock_creep_distance:
            heading_limit = self.dock_near_cross_track_heading_limit
            angular_gain = self.dock_near_heading_gain
            angular_limit = self.dock_near_max_angular_speed
        if axis_lock_active:
            heading_correction = 0.0
        else:
            heading_correction = max(
                -heading_limit,
                min(heading_limit, heading_correction),
            )
        desired_heading = self._normalize_angle(base_heading - heading_correction)
        yaw_error = self._normalize_angle(desired_heading - current_yaw)
        angular_z = max(
            -angular_limit,
            min(angular_limit, angular_gain * yaw_error),
        )

        self._log_heading_once_per_second(
            namespace=namespace,
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            axis_heading=axis_heading,
            axis_error=axis_error,
            target_heading=desired_heading,
            yaw_error=yaw_error,
            distance_to_blocker=max(along_remaining, 0.0),
            cross_track_error=cross_track_error,
        )
        self._publish_cmd_vel(namespace, linear_x, angular_z)

    def _pre_lift_stop_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        self._publish_cmd_vel(namespace, 0.0, 0.0)
        start_time = self._docking_state.get("pre_lift_stop_start_time")
        if start_time is None:
            self._docking_state["pre_lift_stop_start_time"] = self.get_clock().now()
            return
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed < self.pre_lift_stop_duration_sec:
            return
        lift_target_position = self._docking_state.get(
            "lift_target_position",
            self.lift_target_position,
        )
        lift_action_label = (
            "lift lowering"
            if lift_target_position <= self.lift_completion_tolerance
            else "lift raising"
        )
        self._log_robot_info(
            namespace,
            "Pre-lift stop hold finished (%.2fs of zero cmd). Starting final in-place alignment before %s."
            % (self.pre_lift_stop_duration_sec, lift_action_label),
        )
        self._docking_state["phase"] = "lift_align"
        self._docking_state["pre_lift_stop_start_time"] = None
        self._lift_align_step(current_pose)

    def _lift_align_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        _, _, current_yaw = current_pose
        target_heading = self._docking_state["target_heading"]
        lift_target_position = self._docking_state.get(
            "lift_target_position",
            self.lift_target_position,
        )
        lift_action_label = (
            "lift lowering"
            if lift_target_position <= self.lift_completion_tolerance
            else "lift raising"
        )
        yaw_error = self._normalize_angle(target_heading - current_yaw)
        effective_gain = self._compute_lift_align_heading_gain(yaw_error)

        now = self.get_clock().now()
        last_log_time = self._docking_state.get("last_lift_align_log_time")
        if last_log_time is None or (now - last_log_time).nanoseconds / 1e9 >= 1.0:
            self._docking_state["last_lift_align_log_time"] = now
            self._log_robot_info(
                namespace,
                "Lift alignment state before %s: current_yaw=%.3f axis_heading=%.3f axis_error=%.3f brake_tolerance=%.3f completion_tolerance=%.3f gain=%.3f"
                % (
                    lift_action_label,
                    current_yaw,
                    target_heading,
                    yaw_error,
                    self.lift_align_tolerance,
                    self.lift_align_completion_tolerance,
                    effective_gain,
                ),
            )

        if abs(yaw_error) <= self.lift_align_completion_tolerance:
            self._publish_stop(namespace)
            self._log_robot_info(
                namespace,
                "Lift alignment finished for %s at yaw=%.3f target=%.3f within strict completion tolerance %.3f. Waiting for %.1fs of stable stop before commanding the lift."
                % (
                    lift_action_label,
                    current_yaw,
                    target_heading,
                    self.lift_align_completion_tolerance,
                    self.lift_settle_sec,
                ),
            )
            self._docking_state["phase"] = "settle"
            self._docking_state["settle_reference_pose"] = current_pose
            self._docking_state["settle_stable_since"] = self.get_clock().now()
            return

        self._run_forward_in_place_rotation_step(
            namespace=namespace,
            phase="lift_align",
            current_pose=current_pose,
            target_heading=target_heading,
            stop_tolerance=self.lift_align_tolerance,
            gain=effective_gain,
            max_speed=self.lift_align_max_angular_speed,
            reason="Final in-place heading alignment before %s" % lift_action_label,
        )

    def _compute_lift_align_heading_gain(self, yaw_error: float) -> float:
        min_gain = self.lift_align_heading_gain
        max_gain = max(min_gain, self.lift_align_heading_gain_max)
        ramp_error = max(self.lift_align_deadband, self.lift_align_heading_gain_ramp_error)
        error_mag = abs(yaw_error)
        if max_gain <= min_gain or ramp_error <= self.lift_align_deadband:
            return min_gain

        normalized = (error_mag - self.lift_align_deadband) / (
            ramp_error - self.lift_align_deadband
        )
        normalized = max(0.0, min(1.0, normalized))
        return min_gain + ((max_gain - min_gain) * normalized)

    def _settle_before_lift_step(self) -> None:
        namespace = self._docking_state["namespace"]
        self._publish_stop(namespace)
        current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
        if current_pose is None:
            return

        now = self.get_clock().now()
        settle_reference_pose = self._docking_state.get("settle_reference_pose")
        settle_stable_since = self._docking_state.get("settle_stable_since")
        if settle_reference_pose is None or settle_stable_since is None:
            self._docking_state["settle_reference_pose"] = current_pose
            self._docking_state["settle_stable_since"] = now
            return

        settle_dx = current_pose[0] - settle_reference_pose[0]
        settle_dy = current_pose[1] - settle_reference_pose[1]
        settle_drift = math.hypot(settle_dx, settle_dy)
        settle_yaw_drift = abs(
            self._normalize_angle(current_pose[2] - settle_reference_pose[2])
        )
        if (
            settle_drift > self.lift_precheck_position_tolerance
            or settle_yaw_drift > self.lift_precheck_yaw_tolerance
        ):
            self._docking_state["settle_reference_pose"] = current_pose
            self._docking_state["settle_stable_since"] = now
            self._log_robot_info(
                namespace,
                "Lift precheck reset: drift=%.3f m yaw_drift=%.3f rad exceeded stop tolerances (%.3f m, %.3f rad). Waiting for a fresh stable stop."
                % (
                    settle_drift,
                    settle_yaw_drift,
                    self.lift_precheck_position_tolerance,
                    self.lift_precheck_yaw_tolerance,
                ),
            )
            return

        elapsed = (now - settle_stable_since).nanoseconds / 1e9
        if elapsed < self.lift_settle_sec:
            return

        self._log_robot_info(
            namespace,
            "Stable stop confirmed for %.1fs (drift=%.3f m yaw_drift=%.3f rad). Starting lift phase."
            % (
                self.lift_settle_sec,
                settle_drift,
                settle_yaw_drift,
            ),
        )
        self._docking_state["phase"] = "lift"
        self._docking_state["last_lift_publish_time"] = None
        self._lift_step()

    def _lift_step(self) -> None:
        namespace = self._docking_state["namespace"]
        now = self.get_clock().now()
        lift_requested = self._docking_state.get("lift_requested", False)
        lift_target_position = self._docking_state.get("lift_target_position", self.lift_target_position)
        lifting_upward = lift_target_position > self.lift_completion_tolerance
        is_amr1_relocation_lowering = (
            not lifting_upward
            and self._mission_state is not None
            and namespace == self._mission_state.get("amr1_namespace")
            and self._docking_state.get("post_reverse_stage") == "amr1_return_home"
        )
        lift_completion_threshold = max(
            0.0,
            lift_target_position - self.lift_completion_tolerance,
        )
        lower_completion_threshold = max(0.0, self.lift_lower_completion_tolerance)

        if not lift_requested:
            self._write_lift_cmd(namespace, lift_target_position)
            self._docking_state["lift_requested"] = True
            self._docking_state["lift_request_time"] = now
            self._docking_state["last_lift_state_log_time"] = None
            self._docking_state["lift_peak_position"] = 0.0
            self._docking_state["lift_lowest_position"] = None
            if lifting_upward:
                self._set_robot_mission_state(
                    namespace,
                    "LIFTING_UP",
                    "Lift was commanded upward and the pallet is being picked while the AMR prepares to reverse.",
                )
                if (
                    self._mission_state is not None
                    and namespace == self._mission_state.get("amr2_namespace")
                ):
                    self._mission_state["relocate"] = True
                    self._log_robot_info(
                        namespace,
                        "AMR2 lift_up triggered relocate=True; subsequent AMR1 docking must enter RELOCATING flow.",
                    )
            else:
                self._set_robot_mission_state(
                    namespace,
                    "LOWERING",
                    "Lift was commanded down to place the pallet into the slot.",
                )
            self._log_robot_info(
                namespace,
                "Started lift phase for %s -> file command %s (%s=%.3f). Lift verification timer starts now."
                % (
                    self._docking_state["robot"].prim_path,
                    self.lift_cmd_file,
                    self.lift_joint_name,
                    lift_target_position,
                ),
            )
            return

        current_lift = self._read_lift_state(namespace)
        peak_lift = float(self._docking_state.get("lift_peak_position", 0.0))
        lowest_lift = self._docking_state.get("lift_lowest_position")
        if current_lift is not None:
            peak_lift = max(peak_lift, current_lift)
            self._docking_state["lift_peak_position"] = peak_lift
            if lowest_lift is None:
                lowest_lift = current_lift
            else:
                lowest_lift = min(float(lowest_lift), current_lift)
            self._docking_state["lift_lowest_position"] = lowest_lift
        last_log_time = self._docking_state.get("last_lift_state_log_time")
        if current_lift is not None:
            if last_log_time is None or (now - last_log_time).nanoseconds / 1e9 >= 1.0:
                self._docking_state["last_lift_state_log_time"] = now
                if lifting_upward:
                    self._log_robot_info(
                        namespace,
                        "Lift state: current=%.4f target=%.4f complete_at>=%.4f"
                        % (
                            current_lift,
                            lift_target_position,
                            lift_completion_threshold,
                        ),
                    )
                else:
                    self._log_robot_info(
                        namespace,
                        "Lift state: current=%.4f target=%.4f complete_at<=%.4f"
                        % (
                            current_lift,
                            lift_target_position,
                            lower_completion_threshold,
                        ),
                    )
            lift_complete = False
            if lifting_upward:
                lift_complete = current_lift >= lift_completion_threshold
            else:
                lift_complete = current_lift <= lower_completion_threshold
            if lift_complete:
                if (
                    is_amr1_relocation_lowering
                ):
                    self._record_result_event(
                        "amr1_relocation_placed_time",
                        "AMR1 finished lowering the blocker pallet into the former target pallet slot.",
                    )
                self._log_robot_info(
                    namespace,
                    "Lift phase verified: current=%.4f target=%.4f threshold=%.4f. Starting reverse phase immediately."
                    % (
                        current_lift,
                        lift_target_position,
                        lift_completion_threshold if lifting_upward else lower_completion_threshold,
                    ),
                )
                self._docking_state["phase"] = "reverse"
                self._docking_state["start_pose"] = self._latest_amcl_pose_by_namespace[namespace]
                self._docking_state["carrying_load"] = lift_target_position > self.lift_state_tolerance
                if not lifting_upward:
                    self._set_robot_mission_state(
                        namespace,
                        "RETURNING",
                        "Lift was lowered successfully. The AMR is reversing back toward the shared pre_dock.",
                    )
                elif self._mission_state is not None:
                    self._set_robot_mission_state(
                        namespace,
                        "TRANSPORTING",
                        "Lift was raised successfully. The AMR is reversing back toward the shared pre_dock with the lifted pallet.",
                    )
                return

        request_time = self._docking_state.get("lift_request_time")
        if request_time is not None:
            elapsed = (now - request_time).nanoseconds / 1e9
            if elapsed >= self.lift_verify_timeout_sec:
                best_observed_lift = peak_lift
                if current_lift is not None:
                    best_observed_lift = max(best_observed_lift, current_lift)
                best_observed_lower = lowest_lift
                if current_lift is not None:
                    best_observed_lower = current_lift if best_observed_lower is None else min(float(best_observed_lower), current_lift)
                if lifting_upward:
                    if best_observed_lift >= self.lift_timeout_reverse_min_position:
                        self._log_robot_warning(
                            namespace,
                            "Lift verify timeout after %.1fs, but lift reached %.4f m (threshold %.4f m). Proceeding to reverse phase."
                            % (
                                self.lift_verify_timeout_sec,
                                best_observed_lift,
                                self.lift_timeout_reverse_min_position,
                            ),
                        )
                        self._docking_state["phase"] = "reverse"
                        self._docking_state["start_pose"] = self._latest_amcl_pose_by_namespace[namespace]
                        self._docking_state["carrying_load"] = True
                        self._set_robot_mission_state(
                            namespace,
                            "TRANSPORTING",
                            "Lift verify timed out near the pickup threshold; the AMR is reversing back toward the shared pre_dock with the lifted pallet.",
                        )
                        return
                else:
                    if (
                        best_observed_lower is not None
                        and float(best_observed_lower) <= lower_completion_threshold
                    ):
                        self._log_robot_warning(
                            namespace,
                            "Lift lower verify timeout after %.1fs, but lift descended to %.4f m (threshold %.4f m). Proceeding to reverse phase."
                            % (
                                self.lift_verify_timeout_sec,
                                float(best_observed_lower),
                                lower_completion_threshold,
                            ),
                        )
                        if (
                            namespace == (None if self._mission_state is None else self._mission_state.get("amr1_namespace"))
                            and self._docking_state.get("post_reverse_stage") == "amr1_return_home"
                        ):
                            self._record_result_event(
                                "amr1_relocation_placed_time",
                                "AMR1 finished lowering the blocker pallet into the former target pallet slot.",
                            )
                        self._docking_state["phase"] = "reverse"
                        self._docking_state["start_pose"] = self._latest_amcl_pose_by_namespace[namespace]
                        self._docking_state["carrying_load"] = False
                        self._set_robot_mission_state(
                            namespace,
                            "RETURNING",
                            "Lift lowering timed out near the placement threshold, but the AMR is close enough to reverse back toward the shared pre_dock.",
                        )
                        return
                self._log_robot_warning(
                    namespace,
                    "Lift verify timeout after %.1fs. state_file=%s current=%s"
                    % (
                        self.lift_verify_timeout_sec,
                        self.lift_state_file,
                        "N/A" if current_lift is None else f"{current_lift:.4f}",
                    ),
                )
                self._log_robot_warning(
                    namespace,
                    "Best observed lift during the %.1fs window was %.4f m and lowest observed lift was %s, but the completion threshold is %s%.4f m."
                    % (
                        self.lift_verify_timeout_sec,
                        best_observed_lift,
                        "N/A" if best_observed_lower is None else f"{float(best_observed_lower):.4f}",
                        ">=" if lifting_upward else "<=",
                        lift_completion_threshold if lifting_upward else lower_completion_threshold,
                    ),
                )
                self._log_robot_warning(
                    namespace,
                    "Run isaac_lift_bridge.py in Isaac Sim Script Editor and verify Dynamic Control can find lift_joint.",
                )
                self._log_robot_warning(
                    namespace,
                    "Lift verify failed but proceeding to reverse phase so the AMR can leave the slot.",
                )
                if (
                    not lifting_upward
                    and namespace == (None if self._mission_state is None else self._mission_state.get("amr1_namespace"))
                    and self._docking_state.get("post_reverse_stage") == "amr1_return_home"
                ):
                    self._record_result_event(
                        "amr1_relocation_placed_time",
                        "AMR1 lift lower verification failed; proceeding to reverse anyway to clear the slot.",
                    )
                self._docking_state["phase"] = "reverse"
                self._docking_state["start_pose"] = self._latest_amcl_pose_by_namespace[namespace]
                self._docking_state["carrying_load"] = lifting_upward
                self._set_robot_mission_state(
                    namespace,
                    "TRANSPORTING" if lifting_upward else "RETURNING",
                    "Lift verify failed; reversing to clear the slot regardless.",
                )

    def _compute_ramped_reverse_speed(self, ramp_key: str) -> float:
        if self._docking_state.get(ramp_key) is None:
            self._docking_state[ramp_key] = self.get_clock().now()
        elapsed_sec = (
            self.get_clock().now() - self._docking_state[ramp_key]
        ).nanoseconds / 1e9
        progress = min(1.0, elapsed_sec / max(self.reverse_ramp_duration_sec, 1e-6))
        ramped = self.reverse_ramp_start_speed + (
            self.reverse_speed - self.reverse_ramp_start_speed
        ) * progress
        return min(self.reverse_speed, ramped)

    def _apply_terminal_linear_brake(self, speed: float, distance_remaining: float) -> float:
        if distance_remaining <= 0.0:
            return 0.0

        braked_speed = speed
        if distance_remaining < self.manual_terminal_brake_distance:
            brake_progress = max(
                0.0,
                min(
                    1.0,
                    distance_remaining / max(self.manual_terminal_brake_distance, 1e-6),
                ),
            )
            brake_scale = brake_progress ** 2
            brake_target = self.manual_terminal_creep_speed + (
                speed - self.manual_terminal_creep_speed
            ) * brake_scale
            braked_speed = min(braked_speed, brake_target)

        if distance_remaining < self.manual_terminal_final_brake_distance:
            final_progress = max(
                0.0,
                min(
                    1.0,
                    distance_remaining
                    / max(self.manual_terminal_final_brake_distance, 1e-6),
                ),
            )
            final_scale = final_progress ** 2
            final_target = self.manual_terminal_final_speed + (
                self.manual_terminal_creep_speed - self.manual_terminal_final_speed
            ) * final_scale
            braked_speed = min(braked_speed, final_target)

        return max(self.manual_terminal_final_speed, braked_speed)

    def _reverse_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        robot = self._docking_state["robot"]
        blocker = self._docking_state["blocker"]
        approach_axis = self._docking_state["approach_axis"]
        approach_sign = self._docking_state["approach_sign"]
        current_x, current_y, current_yaw = current_pose
        pre_dock_pose = self._docking_state["pre_dock_pose"]
        distance_to_pre_dock = math.hypot(
            current_x - pre_dock_pose.x,
            current_y - pre_dock_pose.y,
        )
        if approach_axis == "x":
            along_signed = approach_sign * (current_x - pre_dock_pose.x)
        else:
            along_signed = approach_sign * (current_y - pre_dock_pose.y)
        if along_signed <= self.reverse_arrive_tolerance:
            self._publish_stop(namespace)
            if (
                self._docking_state.get("post_reverse_stage") == "amr1_return_home"
                and self._mission_state is not None
                and namespace == self._mission_state.get("amr1_namespace")
            ):
                self.get_logger().info(
                    "AMR1 returned to target pre-dock after blocker re-placement. Sending Nav2 goal to AMR1 initial position from pre-dock directly."
                )
                self._manual_control_namespaces.discard(namespace)
                self._docking_state = None
                self._send_amr1_home_goal(current_pose)
                return
            if (
                self._mission_state is not None
                and namespace == self._mission_state.get("amr1_namespace")
                and not self._mission_state.get("amr1_retreat_sent")
                and self._mission_state.get("amr2_robot") is not None
            ):
                retreat_pose, retreat_yaw = self._compute_amr1_retreat_pose(current_pose)
                self._mission_state["amr1_retreat_sent"] = True
                self._mission_state["amr1_departed_pre_dock_with_load"] = True
                self.get_logger().info(
                    "AMR1 returned to pre-dock with blocker pallet. Starting reverse stay_zone phase toward (%.3f, %.3f), reverse_heading=%.3f while keeping lift raised."
                    % (
                        retreat_pose.x,
                        retreat_pose.y,
                        retreat_yaw,
                    )
                )
                self._set_robot_mission_state(
                    namespace,
                    "ALIGNING_TO_STAY",
                    "AMR1 returned to the shared pre_dock with the lifted blocker pallet and is rotating to face stay_zone.",
                )
                self._docking_state["phase"] = "stay_align"
                self._docking_state["stay_zone_pose"] = retreat_pose
                self._docking_state["stay_zone_heading"] = retreat_yaw
                self._docking_state["stay_zone_start_pose"] = current_pose
                self._stay_align_step(current_pose)
                return
            if (
                self._mission_state is not None
                and namespace == self._mission_state.get("amr2_namespace")
                and not self._mission_state.get("amr2_packing_sent")
            ):
                self._record_result_event(
                    "amr2_returned_pre_dock_with_target_time",
                    "AMR2 returned to the shared pre_dock after lifting the target pallet.",
                )
                packing_pose = self._compute_packing_station_pose()
                packing_heading = math.atan2(
                    packing_pose.y - current_pose[1],
                    packing_pose.x - current_pose[0],
                )
                self._mission_state["amr2_packing_sent"] = True
                self.get_logger().info(
                    "AMR2 returned to pre-dock with target pallet. Sending Nav2 goal to PackingStation immediately from pre-dock at (%.3f, %.3f), yaw=%.3f."
                    % (
                        packing_pose.x,
                        packing_pose.y,
                        packing_heading,
                    )
                )
                context = {
                    "stage": "amr2_packing_station",
                    "robot": robot,
                    "namespace": namespace,
                    "blocker": blocker,
                    "target": blocker,
                    "pre_dock_process": PreDockProcessResult(
                        blocker_name=blocker.name,
                        target_name=blocker.name,
                        approach_axis=self._docking_state["approach_axis"],
                        approach_sign=self._docking_state["approach_sign"],
                        pre_dock_pose=packing_pose,
                        pre_dock_yaw=packing_heading,
                    ),
                }
                self._manual_control_namespaces.discard(namespace)
                self._docking_state = None
                self._send_nav_goal(context)
                return
            self.get_logger().info(
                "Reverse phase finished at pre-dock distance %.3f m. Docking sequence completed."
                % distance_to_pre_dock
            )
            self._manual_control_namespaces.discard(namespace)
            self._docking_state = None
            return

        direction_x = 0.0
        direction_y = 0.0
        if approach_axis == "x":
            direction_x = float(approach_sign)
        else:
            direction_y = float(approach_sign)

        base_heading = self._docking_state["target_heading"]
        relative_x = current_x - blocker.pose.x
        relative_y = current_y - blocker.pose.y
        cross_track_error = (-direction_y * relative_x) + (direction_x * relative_y)
        heading_correction = math.atan(
            self.dock_cross_track_heading_gain * cross_track_error
        )
        heading_limit = self.dock_near_cross_track_heading_limit
        heading_correction = max(
            -heading_limit,
            min(heading_limit, heading_correction),
        )
        desired_heading = self._normalize_angle(base_heading + heading_correction)
        yaw_error = self._normalize_angle(desired_heading - current_yaw)
        angular_z = max(
            -self.dock_near_max_angular_speed,
            min(self.dock_near_max_angular_speed, self.dock_near_heading_gain * yaw_error),
        )

        ramped_speed = self._compute_ramped_reverse_speed("reverse_ramp_start_time")
        if along_signed < self.reverse_brake_distance:
            brake_progress = max(
                0.0,
                min(1.0, along_signed / max(self.reverse_brake_distance, 1e-6)),
            )
            speed_scale = brake_progress ** self.reverse_brake_exponent
            braked_target = self.reverse_creep_speed + (
                ramped_speed - self.reverse_creep_speed
            ) * speed_scale
            ramped_speed = max(self.reverse_creep_speed, min(ramped_speed, braked_target))
        ramped_speed = self._apply_terminal_linear_brake(ramped_speed, max(along_signed, 0.0))
        self._publish_cmd_vel(namespace, -ramped_speed, angular_z)

    def _packing_backoff_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        robot = self._docking_state["robot"]
        blocker = self._docking_state["blocker"]
        start_pose = self._docking_state.get("packing_backoff_start_pose", current_pose)
        traveled = math.hypot(
            current_pose[0] - start_pose[0],
            current_pose[1] - start_pose[1],
        )
        base_heading = self._docking_state["target_heading"]
        approach_axis = self._docking_state["approach_axis"]
        approach_sign = self._docking_state["approach_sign"]
        direction_x = 0.0
        direction_y = 0.0
        if approach_axis == "x":
            direction_x = float(approach_sign)
        else:
            direction_y = float(approach_sign)
        current_x, current_y, current_yaw = current_pose
        relative_x = current_x - blocker.pose.x
        relative_y = current_y - blocker.pose.y
        cross_track_error = (-direction_y * relative_x) + (direction_x * relative_y)
        heading_correction = math.atan(
            self.dock_cross_track_heading_gain * cross_track_error
        )
        heading_limit = self.dock_near_cross_track_heading_limit
        heading_correction = max(
            -heading_limit,
            min(heading_limit, heading_correction),
        )
        desired_heading = self._normalize_angle(base_heading + heading_correction)
        yaw_error = self._normalize_angle(desired_heading - current_yaw)
        angular_z = max(
            -self.dock_near_max_angular_speed,
            min(self.dock_near_max_angular_speed, self.dock_near_heading_gain * yaw_error),
        )

        if traveled >= self.amr2_packing_backoff_distance:
            self._publish_stop(namespace)
            packing_pose = self._compute_packing_station_pose()
            packing_heading = math.atan2(
                packing_pose.y - current_pose[1],
                packing_pose.x - current_pose[0],
            )
            if self._mission_state is not None:
                self._mission_state["amr2_packing_sent"] = True
            self.get_logger().info(
                "AMR2 finished %.2f m packing backoff. Sending Nav2 goal to PackingStation at (%.3f, %.3f), yaw=%.3f."
                % (
                    traveled,
                    packing_pose.x,
                    packing_pose.y,
                    packing_heading,
                )
            )
            context = {
                "stage": "amr2_packing_station",
                "robot": robot,
                "namespace": namespace,
                "blocker": blocker,
                "target": blocker,
                "pre_dock_process": PreDockProcessResult(
                    blocker_name=blocker.name,
                    target_name=blocker.name,
                    approach_axis=self._docking_state["approach_axis"],
                    approach_sign=self._docking_state["approach_sign"],
                    pre_dock_pose=packing_pose,
                    pre_dock_yaw=packing_heading,
                ),
            }
            self._docking_state = None
            self._send_nav_goal(context)
            return

        ramped_speed = self._compute_ramped_reverse_speed("packing_backoff_ramp_start_time")
        self._publish_cmd_vel(namespace, -ramped_speed, angular_z)

    def _stay_align_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        current_yaw = current_pose[2]
        target_heading = self._docking_state["stay_zone_heading"]
        yaw_error = self._normalize_angle(target_heading - current_yaw)
        if abs(yaw_error) <= self.stay_align_stop_tolerance:
            self._publish_stop(namespace)
            self._docking_state["phase"] = "stay_reverse"
            self._docking_state["stay_zone_start_pose"] = current_pose
            self._log_robot_info(
                namespace,
                "Stay zone alignment finished at yaw=%.3f. Starting reverse to stay_zone."
                % current_yaw,
            )
            self._set_robot_mission_state(
                namespace,
                "MOVING_TO_STAY",
                "AMR1 finished aligning at the shared pre_dock and is reversing toward stay_zone.",
            )
            self._stay_reverse_step(current_pose)
            return
        self._run_reverse_in_place_rotation_step(
            namespace=namespace,
            phase="stay_align",
            current_pose=current_pose,
            target_heading=target_heading,
            stop_tolerance=self.stay_align_brake_tolerance,
            gain=self.stay_align_heading_gain,
            max_speed=self.stay_align_max_angular_speed,
            reason="Align in place toward stay_zone reverse heading",
        )

    def _stay_reverse_step(self, current_pose: tuple[float, float, float]) -> None:
        namespace = self._docking_state["namespace"]
        stay_zone_pose = self._docking_state["stay_zone_pose"]
        stay_zone_start_pose = self._docking_state.get("stay_zone_start_pose", current_pose)
        target_heading = self._docking_state["stay_zone_heading"]
        distance_to_stay_zone = math.hypot(
            current_pose[0] - stay_zone_pose.x,
            current_pose[1] - stay_zone_pose.y,
        )
        target_dx = stay_zone_pose.x - stay_zone_start_pose[0]
        target_dy = stay_zone_pose.y - stay_zone_start_pose[1]
        target_distance = math.hypot(target_dx, target_dy)
        progress_along_target = 0.0
        if target_distance > 1e-6:
            progress_along_target = (
                (current_pose[0] - stay_zone_start_pose[0]) * (target_dx / target_distance)
                + (current_pose[1] - stay_zone_start_pose[1]) * (target_dy / target_distance)
            )
        if (
            distance_to_stay_zone <= self.stay_zone_arrive_tolerance
            or progress_along_target >= (target_distance - self.stay_zone_arrive_tolerance)
        ):
            self._publish_stop(namespace)
            self._log_robot_info(
                namespace,
                "Reached stay_zone at distance %.3f m with %.3f m progress along the planned retreat. AMR2 may resume."
                % (distance_to_stay_zone, progress_along_target),
            )
            if self._mission_state is not None:
                self._mission_state["amr1_retreat_complete"] = True
            self._set_robot_mission_state(
                namespace,
                "WAIT",
                "AMR1 reached stay_zone and is waiting there for AMR2 to clear the shared pre_dock area.",
            )
            self._manual_control_namespaces.discard(namespace)
            self._docking_state = None
            self._maybe_resume_amr2()
            return
        yaw_error = self._normalize_angle(target_heading - current_pose[2])
        angular_z = max(
            -self.dock_near_max_angular_speed,
            min(self.dock_near_max_angular_speed, self.dock_near_heading_gain * yaw_error),
        )
        ramped_speed = self._compute_ramped_reverse_speed("stay_reverse_ramp_start_time")
        if distance_to_stay_zone < self.stay_reverse_brake_distance:
            brake_progress = max(
                0.0,
                min(1.0, distance_to_stay_zone / max(self.stay_reverse_brake_distance, 1e-6)),
            )
            braked_target = self.stay_reverse_creep_speed + (
                ramped_speed - self.stay_reverse_creep_speed
            ) * brake_progress
            ramped_speed = max(self.stay_reverse_creep_speed, min(ramped_speed, braked_target))
        ramped_speed = self._apply_terminal_linear_brake(ramped_speed, distance_to_stay_zone)
        self._publish_cmd_vel(namespace, -ramped_speed, angular_z)

    def _reset_robot_log_files(self) -> None:
        try:
            os.makedirs(self.robot_log_dir, exist_ok=True)
            for path in self._robot_log_paths.values():
                with open(path, "w", encoding="utf-8") as handle:
                    handle.write("")
            self.get_logger().info(
                "Reset AMR log files at startup in %s" % self.robot_log_dir
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Failed to reset robot log files in %s: %s"
                % (self.robot_log_dir, exc)
            )

    def _reset_result_log_files(self) -> None:
        try:
            os.makedirs(self.result_dir, exist_ok=True)
            with open(self._result_log_path, "w", encoding="utf-8") as handle:
                handle.write("# Result Log\n")
                handle.write("# 이동 시간: AMR1 MOVE 시작 -> AMR1 첫 shared pre_dock 도착\n")
                handle.write("# DIGGING 시간: AMR1 DIGGING 시작 -> AMR2 target pallet lift 후 shared pre_dock 복귀\n")
                handle.write("# 전체 시간: AMR1 MOVE 시작 -> AMR1 blocker pallet 재배치 완료\n")
            self._append_result_log("INFO", "Result tracking log initialized.")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Failed to reset result log file in %s: %s"
                % (self.result_dir, exc)
            )

    def _append_robot_log(self, namespace: str, level: str, message: str) -> None:
        path = self._robot_log_paths.get(namespace)
        if path is None:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            timestamp = self.get_clock().now().nanoseconds / 1e9
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(f"[{timestamp:10.3f}] [{level}] {message}\n")
        except Exception:
            pass

    def _append_result_log(self, level: str, message: str) -> None:
        try:
            os.makedirs(os.path.dirname(self._result_log_path), exist_ok=True)
            timestamp = self.get_clock().now().nanoseconds / 1e9
            with open(self._result_log_path, "a", encoding="utf-8") as handle:
                handle.write(f"[{timestamp:10.3f}] [{level}] {message}\n")
        except Exception:
            pass

    def _log_robot_info(self, namespace: str, message: str) -> None:
        self.get_logger().info(message)
        self._append_robot_log(namespace, "INFO", message)

    def _log_robot_warning(self, namespace: str, message: str) -> None:
        self.get_logger().warning(message)
        self._append_robot_log(namespace, "WARN", message)

    def _get_cmd_vel_publisher(self, namespace: str):
        publisher = self._cmd_vel_publishers.get(namespace)
        if publisher is None:
            publisher = self.create_publisher(Twist, f"/{namespace}/cmd_vel", 10)
            self._cmd_vel_publishers[namespace] = publisher
        return publisher

    def _get_lift_trigger_publisher(self, namespace: str):
        publisher = self._lift_trigger_publishers.get(namespace)
        if publisher is None:
            publisher = self.create_publisher(
                Bool,
                f"/{namespace}/{self.lift_trigger_topic_suffix}",
                10,
            )
            self._lift_trigger_publishers[namespace] = publisher
        return publisher

    def _publish_cmd_vel(self, namespace: str, linear_x: float, angular_z: float) -> None:
        publisher = self._get_cmd_vel_publisher(namespace)
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        publisher.publish(msg)
        self._log_manual_cmd_publish(namespace, msg.linear.x, msg.angular.z)

    def _publish_stop(self, namespace: str) -> None:
        self._publish_cmd_vel(namespace, 0.0, 0.0)

    def _write_lift_cmd(self, namespace: str, target_height: float) -> None:
        try:
            cmd_dir = os.path.dirname(self.lift_cmd_file)
            if cmd_dir:
                os.makedirs(cmd_dir, exist_ok=True)
            try:
                with open(self.lift_cmd_file, "r", encoding="utf-8") as f:
                    cmds = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                cmds = {}
            cmds[namespace] = target_height
            with open(self.lift_cmd_file, "w", encoding="utf-8") as f:
                json.dump(cmds, f)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Failed to write lift command file %s: %s" % (self.lift_cmd_file, exc)
            )

    def _reset_lift_command_files(self) -> None:
        namespaces = set(NAV2_NAMESPACE_BY_ROBOT.values())
        try:
            cmd_dir = os.path.dirname(self.lift_cmd_file)
            if cmd_dir:
                os.makedirs(cmd_dir, exist_ok=True)
            reset_cmds = {namespace: 0.0 for namespace in namespaces}
            with open(self.lift_cmd_file, "w", encoding="utf-8") as f:
                json.dump(reset_cmds, f)

            state_dir = os.path.dirname(self.lift_state_file)
            if state_dir:
                os.makedirs(state_dir, exist_ok=True)
            reset_states = {namespace: 0.0 for namespace in namespaces}
            with open(self.lift_state_file, "w", encoding="utf-8") as f:
                json.dump(reset_states, f)

            self.get_logger().info(
                "Reset lift command/state files to 0.0 at startup: %s, %s"
                % (self.lift_cmd_file, self.lift_state_file)
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Failed to reset lift command/state files at startup: %s" % exc
            )

    def _read_lift_state(self, namespace: str):
        try:
            with open(self.lift_state_file, "r", encoding="utf-8") as f:
                states = json.load(f)
            value = states.get(namespace)
            if value is None:
                return None
            return float(value)
        except (FileNotFoundError, json.JSONDecodeError, ValueError):
            return None

    def _log_heading_once_per_second(
        self,
        namespace: str,
        current_x: float,
        current_y: float,
        current_yaw: float,
        axis_heading: float,
        axis_error: float,
        target_heading: float,
        yaw_error: float,
        distance_to_blocker: float,
        cross_track_error: float,
    ) -> None:
        now = self.get_clock().now()
        last_log_time = self._docking_state.get("last_heading_log_time")
        if last_log_time is not None:
            elapsed = (now - last_log_time).nanoseconds / 1e9
            if elapsed < 1.0:
                return
        self._docking_state["last_heading_log_time"] = now
        self.get_logger().info(
            "Docking state for %s: pos=(%.3f, %.3f) current_yaw=%.3f axis_heading=%.3f "
            "axis_error=%.3f target_yaw=%.3f yaw_error=%.3f along=%.3f cross=%.3f phase=%s lift_done=%s"
            % (
                namespace,
                current_x,
                current_y,
                current_yaw,
                axis_heading,
                axis_error,
                target_heading,
                yaw_error,
                distance_to_blocker,
                cross_track_error,
                self._docking_state["phase"],
                self._docking_state["lift_publish_remaining"] <= 0,
            )
        )
        self._append_robot_log(
            namespace,
            "INFO",
            "Docking state: pos=(%.3f, %.3f) current_yaw=%.3f axis_heading=%.3f axis_error=%.3f target_yaw=%.3f yaw_error=%.3f along=%.3f cross=%.3f phase=%s lift_done=%s"
            % (
                current_x,
                current_y,
                current_yaw,
                axis_heading,
                axis_error,
                target_heading,
                yaw_error,
                distance_to_blocker,
                cross_track_error,
                self._docking_state["phase"],
                self._docking_state["lift_publish_remaining"] <= 0,
            ),
        )

    def _log_pre_dock_tracking_once_per_second(self) -> None:
        if self._mission_state is None:
            return

        now = self.get_clock().now()
        last_log_time = self._mission_state.get("last_pre_dock_tracking_log_time")
        if last_log_time is not None and (now - last_log_time).nanoseconds / 1e9 < 1.0:
            return
        self._mission_state["last_pre_dock_tracking_log_time"] = now

        for namespace_key, process_key, label in (
            ("amr1_namespace", "blocker_pre_dock_process", "amr1_pre_dock"),
            ("amr2_namespace", "target_pre_dock_process", "amr2_pre_dock"),
        ):
            namespace = self._mission_state.get(namespace_key)
            pre_dock_process = self._mission_state.get(process_key)
            if namespace is None or pre_dock_process is None:
                continue

            current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
            if current_pose is None:
                self.get_logger().info(
                    "%s tracking for %s: current_pose unavailable (stage=nav2_pre_dock)"
                    % (label, namespace)
                )
                continue

            target_pose = pre_dock_process.pre_dock_pose
            target_yaw = pre_dock_process.pre_dock_yaw
            distance = math.hypot(
                current_pose[0] - target_pose.x,
                current_pose[1] - target_pose.y,
            )
            yaw_error = self._normalize_angle(target_yaw - current_pose[2])
            mission_state = self._get_robot_mission_state(namespace)
            phase = "manual_docking" if (
                self._docking_state is not None
                and self._docking_state.get("namespace") == namespace
            ) else "nav2_pre_dock"
            self.get_logger().info(
                "%s tracking for %s: state=%s phase=%s current=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) distance=%.3f yaw_error=%.3f"
                % (
                    label,
                    namespace,
                    "UNSET" if mission_state is None else mission_state,
                    phase,
                    current_pose[0],
                    current_pose[1],
                    current_pose[2],
                    target_pose.x,
                    target_pose.y,
                    target_yaw,
                    distance,
                    yaw_error,
                )
            )
            self._append_robot_log(
                namespace,
                "INFO",
                "%s tracking: state=%s phase=%s current=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) distance=%.3f yaw_error=%.3f"
                % (
                    label,
                    "UNSET" if mission_state is None else mission_state,
                    phase,
                    current_pose[0],
                    current_pose[1],
                    current_pose[2],
                    target_pose.x,
                    target_pose.y,
                    target_yaw,
                    distance,
                    yaw_error,
                ),
            )

    def _log_manual_phase_state_once_per_second(
        self,
        current_pose: tuple[float, float, float],
    ) -> None:
        if self._docking_state is None:
            return

        now = self.get_clock().now()
        last_log_time = self._docking_state.get("last_manual_phase_state_log_time")
        if last_log_time is not None and (now - last_log_time).nanoseconds / 1e9 < 1.0:
            return
        self._docking_state["last_manual_phase_state_log_time"] = now

        namespace = self._docking_state["namespace"]
        pre_dock_pose = self._docking_state.get("pre_dock_pose")
        target_heading = self._get_active_phase_target_heading()
        blocker = self._docking_state.get("blocker")
        blocker_x = float("nan") if blocker is None else blocker.pose.x
        blocker_y = float("nan") if blocker is None else blocker.pose.y
        pre_dock_distance = float("nan")
        yaw_error = float("nan")
        if pre_dock_pose is not None:
            pre_dock_distance = math.hypot(
                current_pose[0] - pre_dock_pose.x,
                current_pose[1] - pre_dock_pose.y,
            )
        if target_heading is not None:
            yaw_error = self._normalize_angle(target_heading - current_pose[2])

        self.get_logger().info(
            "Manual docking state for %s: phase=%s current=(%.3f, %.3f, %.3f) target_heading=%s yaw_error=%s pre_dock_distance=%s blocker=(%.3f, %.3f)"
            % (
                namespace,
                self._docking_state.get("phase", "unknown"),
                current_pose[0],
                current_pose[1],
                current_pose[2],
                "N/A" if target_heading is None else f"{target_heading:.3f}",
                "N/A" if math.isnan(yaw_error) else f"{yaw_error:.3f}",
                "N/A" if math.isnan(pre_dock_distance) else f"{pre_dock_distance:.3f}",
                blocker_x,
                blocker_y,
            )
        )
        self._append_robot_log(
            namespace,
            "INFO",
            "Manual docking state: phase=%s current=(%.3f, %.3f, %.3f) target_heading=%s yaw_error=%s pre_dock_distance=%s blocker=(%.3f, %.3f)"
            % (
                self._docking_state.get("phase", "unknown"),
                current_pose[0],
                current_pose[1],
                current_pose[2],
                "N/A" if target_heading is None else f"{target_heading:.3f}",
                "N/A" if math.isnan(yaw_error) else f"{yaw_error:.3f}",
                "N/A" if math.isnan(pre_dock_distance) else f"{pre_dock_distance:.3f}",
                blocker_x,
                blocker_y,
            ),
        )

    def _get_active_phase_target_heading(self) -> float | None:
        if self._docking_state is None:
            return None
        phase = self._docking_state.get("phase")
        if phase == "stay_align":
            return self._docking_state.get("stay_zone_heading")
        return self._docking_state.get("target_heading")

    def _log_manual_rotation_once_per_second(
        self,
        namespace: str,
        phase: str,
        current_pose: tuple[float, float, float],
        target_heading: float,
        yaw_error: float,
        angular_z: float,
        reason: str,
    ) -> None:
        now = self.get_clock().now()
        last_log_time = self._docking_state.get("last_manual_rotation_log_time")
        if last_log_time is not None:
            elapsed = (now - last_log_time).nanoseconds / 1e9
            if elapsed < 1.0:
                return
        self._docking_state["last_manual_rotation_log_time"] = now
        current_x, current_y, current_yaw = current_pose
        blocker = self._docking_state.get("blocker")
        blocker_x = float("nan") if blocker is None else blocker.pose.x
        blocker_y = float("nan") if blocker is None else blocker.pose.y
        pre_dock_pose = self._docking_state.get("pre_dock_pose")
        pre_dock_x = float("nan") if pre_dock_pose is None else pre_dock_pose.x
        pre_dock_y = float("nan") if pre_dock_pose is None else pre_dock_pose.y
        self.get_logger().info(
            "Manual rotation for %s: phase=%s current=(%.3f, %.3f, %.3f) target_heading=%.3f yaw_error=%.3f cmd_angular=%.3f blocker=(%.3f, %.3f) pre_dock=(%.3f, %.3f) reason=%s"
            % (
                namespace,
                phase,
                current_x,
                current_y,
                current_yaw,
                target_heading,
                yaw_error,
                angular_z,
                blocker_x,
                blocker_y,
                pre_dock_x,
                pre_dock_y,
                reason,
            )
        )
        self._append_robot_log(
            namespace,
            "INFO",
            "Manual rotation: phase=%s current=(%.3f, %.3f, %.3f) target_heading=%.3f yaw_error=%.3f cmd_angular=%.3f blocker=(%.3f, %.3f) pre_dock=(%.3f, %.3f) reason=%s"
            % (
                phase,
                current_x,
                current_y,
                current_yaw,
                target_heading,
                yaw_error,
                angular_z,
                blocker_x,
                blocker_y,
                pre_dock_x,
                pre_dock_y,
                reason,
            ),
        )

    def _log_manual_cmd_publish(self, namespace: str, linear_x: float, angular_z: float) -> None:
        if self._docking_state is None or self._docking_state.get("namespace") != namespace:
            return

        now = self.get_clock().now()
        current_pose = self._latest_amcl_pose_by_namespace.get(namespace)
        phase = self._docking_state.get("phase", "unknown")
        target_heading = self._get_active_phase_target_heading()
        blocker = self._docking_state.get("blocker")
        blocker_x = float("nan") if blocker is None else blocker.pose.x
        blocker_y = float("nan") if blocker is None else blocker.pose.y

        last_log_time = self._docking_state.get("last_manual_cmd_log_time")
        if last_log_time is None or (now - last_log_time).nanoseconds / 1e9 >= 1.0:
            self._docking_state["last_manual_cmd_log_time"] = now
            if current_pose is None:
                self.get_logger().info(
                    "Manual cmd for %s: phase=%s cmd=(linear=%.3f, angular=%.3f) target_heading=%s blocker=(%.3f, %.3f) current_pose=unavailable"
                    % (
                        namespace,
                        phase,
                        linear_x,
                        angular_z,
                        "N/A" if target_heading is None else f"{target_heading:.3f}",
                        blocker_x,
                        blocker_y,
                    )
                )
                self._append_robot_log(
                    namespace,
                    "INFO",
                    "Manual cmd: phase=%s cmd=(linear=%.3f, angular=%.3f) target_heading=%s blocker=(%.3f, %.3f) current_pose=unavailable"
                    % (
                        phase,
                        linear_x,
                        angular_z,
                        "N/A" if target_heading is None else f"{target_heading:.3f}",
                        blocker_x,
                        blocker_y,
                    ),
                )
            else:
                yaw_error = None
                if target_heading is not None:
                    yaw_error = self._normalize_angle(target_heading - current_pose[2])
                self.get_logger().info(
                    "Manual cmd for %s: phase=%s current=(%.3f, %.3f, %.3f) target_heading=%s yaw_error=%s cmd=(linear=%.3f, angular=%.3f) blocker=(%.3f, %.3f)"
                    % (
                        namespace,
                        phase,
                        current_pose[0],
                        current_pose[1],
                        current_pose[2],
                        "N/A" if target_heading is None else f"{target_heading:.3f}",
                        "N/A" if yaw_error is None else f"{yaw_error:.3f}",
                        linear_x,
                        angular_z,
                        blocker_x,
                        blocker_y,
                    )
                )
                self._append_robot_log(
                    namespace,
                    "INFO",
                    "Manual cmd: phase=%s current=(%.3f, %.3f, %.3f) target_heading=%s yaw_error=%s cmd=(linear=%.3f, angular=%.3f) blocker=(%.3f, %.3f)"
                    % (
                        phase,
                        current_pose[0],
                        current_pose[1],
                        current_pose[2],
                        "N/A" if target_heading is None else f"{target_heading:.3f}",
                        "N/A" if yaw_error is None else f"{yaw_error:.3f}",
                        linear_x,
                        angular_z,
                        blocker_x,
                        blocker_y,
                    ),
                )

        last_diag_time = self._docking_state.get("last_motion_diag_time")
        last_diag_pose = self._docking_state.get("last_motion_diag_pose")
        last_diag_cmd = self._docking_state.get("last_motion_diag_cmd")
        if (
            current_pose is not None
            and last_diag_time is not None
            and last_diag_pose is not None
            and last_diag_cmd is not None
            and (now - last_diag_time).nanoseconds / 1e9 >= 1.0
        ):
            position_delta = math.hypot(
                current_pose[0] - last_diag_pose[0],
                current_pose[1] - last_diag_pose[1],
            )
            yaw_delta = abs(self._normalize_angle(current_pose[2] - last_diag_pose[2]))
            if (
                abs(last_diag_cmd[0]) > 0.01 or abs(last_diag_cmd[1]) > 0.01
            ) and position_delta < 0.01 and yaw_delta < 0.03:
                self.get_logger().warning(
                    "Manual cmd may not be taking effect for %s: phase=%s previous_cmd=(linear=%.3f, angular=%.3f) pose_change=(%.3f m, %.3f rad) over %.2fs. Check cmd_vel consumer, controller override, TF/pose updates, or collision/cost constraints."
                    % (
                        namespace,
                        phase,
                        last_diag_cmd[0],
                        last_diag_cmd[1],
                        position_delta,
                        yaw_delta,
                        (now - last_diag_time).nanoseconds / 1e9,
                    )
                )
                self._append_robot_log(
                    namespace,
                    "WARN",
                    "Manual cmd may not be taking effect: phase=%s previous_cmd=(linear=%.3f, angular=%.3f) pose_change=(%.3f m, %.3f rad) over %.2fs"
                    % (
                        phase,
                        last_diag_cmd[0],
                        last_diag_cmd[1],
                        position_delta,
                        yaw_delta,
                        (now - last_diag_time).nanoseconds / 1e9,
                    ),
                )

        if current_pose is not None:
            self._docking_state["last_motion_diag_pose"] = current_pose
        self._docking_state["last_motion_diag_time"] = now
        self._docking_state["last_motion_diag_cmd"] = (linear_x, angular_z)

    def _approach_amr2_wait_gate_manually(
        self,
        namespace: str,
        current_pose: tuple[float, float, float],
        pre_dock_pose: Pose3D,
        distance_to_pre_dock: float,
    ) -> None:
        desired_heading = math.atan2(
            pre_dock_pose.y - current_pose[1],
            pre_dock_pose.x - current_pose[0],
        )
        yaw_error = self._normalize_angle(desired_heading - current_pose[2])
        angular_z = max(
            -self.amr2_wait_manual_approach_max_angular_speed,
            min(
                self.amr2_wait_manual_approach_max_angular_speed,
                self.amr2_wait_manual_approach_heading_gain * yaw_error,
            ),
        )
        heading_ratio = max(
            0.0,
            1.0 - (abs(yaw_error) / max(self.amr2_wait_manual_approach_heading_tolerance, 1e-6)),
        )
        linear_x = self.amr2_wait_manual_approach_speed * heading_ratio

        self._log_robot_info(
            namespace,
            "AMR2 wait-gate manual approach: current=(%.3f, %.3f, %.3f) pre_dock=(%.3f, %.3f) distance=%.3f desired_heading=%.3f yaw_error=%.3f cmd=(linear=%.3f, angular=%.3f)"
            % (
                current_pose[0],
                current_pose[1],
                current_pose[2],
                pre_dock_pose.x,
                pre_dock_pose.y,
                distance_to_pre_dock,
                desired_heading,
                yaw_error,
                linear_x,
                angular_z,
            ),
        )
        self._publish_cmd_vel(namespace, linear_x, angular_z)

    def _create_nav_goal(self, goal_x: float, goal_y: float, goal_yaw: float):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.goal_frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(goal_yaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(goal_yaw * 0.5)
        return goal_msg

    def _run_forward_in_place_rotation_step(
        self,
        namespace: str,
        phase: str,
        current_pose: tuple[float, float, float],
        target_heading: float,
        stop_tolerance: float,
        gain: float,
        max_speed: float,
        reason: str,
    ) -> None:
        yaw_error = self._normalize_angle(target_heading - current_pose[2])
        carrying_load = bool(self._docking_state and self._docking_state.get("carrying_load"))
        effective_max_speed = max_speed
        brake_start_override = max(self.angular_brake_start_tolerance * 0.7, stop_tolerance * 2.0)
        brake_exponent_override = 1.5
        min_speed_override = max(self.angular_brake_min_speed, 0.14)
        final_brake_start_override = max(self.angular_brake_final_tolerance * 0.8, stop_tolerance * 1.05)
        final_brake_exponent_override = 1.8
        final_min_speed_override = max(self.angular_brake_final_min_speed, 0.10)
        if phase == "pre_turn_forward":
            min_speed_override = max(min_speed_override, self.pre_turn_min_angular_speed)
            final_min_speed_override = max(
                final_min_speed_override,
                self.pre_turn_min_angular_speed,
            )
            brake_exponent_override = max(brake_exponent_override, 4.0)
            final_brake_exponent_override = max(final_brake_exponent_override, 6.0)
            brake_start_override = max(brake_start_override, stop_tolerance * 5.0)
            final_brake_start_override = max(final_brake_start_override, stop_tolerance * 2.5)
        if phase == "lift_align":
            min_speed_override = max(min_speed_override, self.lift_align_min_angular_speed)
            final_min_speed_override = max(final_min_speed_override, self.lift_align_min_angular_speed)
            effective_max_speed = min(effective_max_speed, max_speed * 0.6)
            brake_exponent_override = max(brake_exponent_override, 5.0)
            final_brake_exponent_override = max(final_brake_exponent_override, 8.0)
            brake_start_override = max(brake_start_override, stop_tolerance * 6.0)
            final_brake_start_override = max(final_brake_start_override, stop_tolerance * 3.0)
        if carrying_load:
            effective_max_speed = max(0.08, max_speed * self.loaded_rotation_speed_scale)
            brake_start_override = max(self.angular_brake_start_tolerance * 1.25, stop_tolerance * 2.5)
            brake_exponent_override = max(self.angular_brake_exponent, 2.6)
            min_speed_override = max(self.angular_brake_min_speed * 0.8, 0.08)
            final_brake_start_override = max(self.angular_brake_final_tolerance * 1.25, stop_tolerance * 1.15)
            final_brake_exponent_override = max(self.angular_brake_final_exponent, 4.4)
            final_min_speed_override = max(self.angular_brake_final_min_speed, 0.03)
            if phase == "pre_turn_forward":
                effective_max_speed = max(
                    self.loaded_pre_turn_min_angular_speed,
                    max_speed * self.loaded_pre_turn_rotation_speed_scale,
                )
                min_speed_override = max(
                    min_speed_override,
                    self.loaded_pre_turn_min_angular_speed,
                )
                final_min_speed_override = max(
                    final_min_speed_override,
                    self.loaded_pre_turn_min_angular_speed,
                )
            if phase == "lift_align":
                min_speed_override = max(
                    min_speed_override,
                    self.lift_align_loaded_min_angular_speed,
                )
                final_min_speed_override = max(
                    final_min_speed_override,
                    self.lift_align_loaded_min_angular_speed,
                )
        angular_z = self._compute_braked_angular_z(
            yaw_error=yaw_error,
            gain=gain,
            max_speed=effective_max_speed,
            stop_tolerance=stop_tolerance,
            brake_start_override=brake_start_override,
            brake_exponent_override=brake_exponent_override,
            min_speed_override=min_speed_override,
            final_brake_start_override=final_brake_start_override,
            final_brake_exponent_override=final_brake_exponent_override,
            final_min_speed_override=final_min_speed_override,
            disable_final_brake=carrying_load,
        )
        if phase == "lift_align" and abs(yaw_error) > stop_tolerance * 1.5:
            breakaway_floor = (
                self.lift_align_loaded_min_angular_speed
                if carrying_load
                else self.lift_align_min_angular_speed
            )
            error_scale = min(1.0, abs(yaw_error) / max(stop_tolerance * 2.0, 1e-6))
            scaled_floor = min(breakaway_floor * error_scale, effective_max_speed)
            if abs(angular_z) < scaled_floor:
                angular_z = math.copysign(scaled_floor, yaw_error)
        if phase == "pre_turn_forward":
            if carrying_load and abs(yaw_error) > self.pre_turn_heading_stop_tolerance:
                fixed_floor = min(self.loaded_pre_turn_min_angular_speed, effective_max_speed)
                if abs(angular_z) < fixed_floor:
                    angular_z = math.copysign(fixed_floor, yaw_error)
            elif abs(yaw_error) > stop_tolerance * 1.5:
                breakaway_floor = self.pre_turn_min_angular_speed
                error_scale = min(1.0, abs(yaw_error) / max(stop_tolerance * 2.0, 1e-6))
                scaled_floor = min(breakaway_floor * error_scale, effective_max_speed)
                if abs(angular_z) < scaled_floor:
                    angular_z = math.copysign(scaled_floor, yaw_error)
        self._log_manual_rotation_once_per_second(
            namespace=namespace,
            phase=phase,
            current_pose=current_pose,
            target_heading=target_heading,
            yaw_error=yaw_error,
            angular_z=angular_z,
            reason=reason,
        )
        self._publish_cmd_vel(namespace, 0.0, angular_z)

    def _run_reverse_in_place_rotation_step(
        self,
        namespace: str,
        phase: str,
        current_pose: tuple[float, float, float],
        target_heading: float,
        stop_tolerance: float,
        gain: float,
        max_speed: float,
        reason: str,
    ) -> None:
        yaw_error = self._normalize_angle(target_heading - current_pose[2])
        carrying_load = bool(self._docking_state and self._docking_state.get("carrying_load"))
        effective_max_speed = max_speed
        brake_start_override = max(self.angular_brake_start_tolerance, stop_tolerance * 4.5)
        brake_exponent_override = max(self.angular_brake_exponent, 2.4)
        min_speed_override = self.angular_brake_min_speed
        final_brake_start_override = None
        final_brake_exponent_override = None
        final_min_speed_override = None
        if carrying_load:
            effective_max_speed = max(0.08, max_speed * self.loaded_rotation_speed_scale)
            brake_start_override = max(self.angular_brake_start_tolerance * 1.35, stop_tolerance * 5.0)
            brake_exponent_override = max(self.angular_brake_exponent, 3.0)
            min_speed_override = max(self.angular_brake_min_speed * 0.75, 0.07)
            final_brake_start_override = max(self.angular_brake_final_tolerance * 1.35, stop_tolerance * 1.2)
            final_brake_exponent_override = max(self.angular_brake_final_exponent, 4.8)
            final_min_speed_override = max(self.angular_brake_final_min_speed, 0.025)
            if phase == "stay_align":
                effective_max_speed = max(
                    self.loaded_stay_align_min_angular_speed,
                    max_speed * self.loaded_stay_align_rotation_speed_scale,
                )
                min_speed_override = max(
                    min_speed_override,
                    self.loaded_stay_align_min_angular_speed,
                )
                final_min_speed_override = max(
                    final_min_speed_override,
                    self.loaded_stay_align_min_angular_speed * 0.7,
                )
        angular_z = self._compute_braked_angular_z(
            yaw_error=yaw_error,
            gain=gain,
            max_speed=effective_max_speed,
            stop_tolerance=stop_tolerance,
            brake_start_override=brake_start_override,
            brake_exponent_override=brake_exponent_override,
            min_speed_override=min_speed_override,
            final_brake_start_override=final_brake_start_override,
            final_brake_exponent_override=final_brake_exponent_override,
            final_min_speed_override=final_min_speed_override,
            disable_final_brake=carrying_load,
        )
        if phase == "stay_align" and abs(yaw_error) > stop_tolerance:
            breakaway_floor = (
                self.loaded_stay_align_min_angular_speed
                if carrying_load
                else self.stay_align_min_angular_speed
            )
            error_scale = min(1.0, abs(yaw_error) / max(stop_tolerance * 2.0, 1e-6))
            scaled_floor = min(breakaway_floor * error_scale, effective_max_speed)
            if abs(angular_z) < scaled_floor:
                angular_z = math.copysign(scaled_floor, yaw_error)
        self._log_manual_rotation_once_per_second(
            namespace=namespace,
            phase=phase,
            current_pose=current_pose,
            target_heading=target_heading,
            yaw_error=yaw_error,
            angular_z=angular_z,
            reason=reason,
        )
        self._publish_cmd_vel(namespace, 0.0, angular_z)

    def _compute_braked_angular_z(
        self,
        yaw_error: float,
        gain: float,
        max_speed: float,
        stop_tolerance: float,
        brake_start_override: float | None = None,
        brake_exponent_override: float | None = None,
        min_speed_override: float | None = None,
        final_brake_start_override: float | None = None,
        final_brake_exponent_override: float | None = None,
        final_min_speed_override: float | None = None,
        disable_final_brake: bool = False,
    ) -> float:
        raw_angular_z = gain * yaw_error
        abs_error = abs(yaw_error)
        limited_max_speed = max_speed
        brake_start_source = self.angular_brake_start_tolerance if brake_start_override is None else brake_start_override
        brake_start = max(brake_start_source, stop_tolerance * 1.5)
        brake_exponent = self.angular_brake_exponent if brake_exponent_override is None else brake_exponent_override
        min_speed = self.angular_brake_min_speed if min_speed_override is None else min_speed_override
        final_brake_start_source = (
            self.angular_brake_final_tolerance
            if final_brake_start_override is None
            else final_brake_start_override
        )
        final_brake_start = max(final_brake_start_source, stop_tolerance * 1.1)
        final_brake_exponent_source = (
            self.angular_brake_final_exponent
            if final_brake_exponent_override is None
            else final_brake_exponent_override
        )
        final_brake_exponent = max(final_brake_exponent_source, 1.0)
        final_min_speed_source = (
            self.angular_brake_final_min_speed
            if final_min_speed_override is None
            else final_min_speed_override
        )
        final_min_speed = min(min_speed, final_min_speed_source)

        if abs_error < brake_start and brake_start > 1e-6 and max_speed > 1e-6:
            progress = max(0.0, min(1.0, abs_error / brake_start))
            speed_scale = progress ** brake_exponent
            min_scale = max(0.0, min(1.0, min_speed / max_speed))
            limited_max_speed = max(min_speed, max_speed * max(min_scale, speed_scale))

        if (
            not disable_final_brake
            and abs_error < final_brake_start
            and final_brake_start > 1e-6
            and max_speed > 1e-6
        ):
            final_progress = max(0.0, min(1.0, abs_error / final_brake_start))
            final_speed_scale = final_progress ** final_brake_exponent
            final_min_scale = max(0.0, min(1.0, final_min_speed / max_speed))
            final_limited_speed = max(
                final_min_speed,
                max_speed * max(final_min_scale, final_speed_scale),
            )
            limited_max_speed = min(limited_max_speed, final_limited_speed)

        if abs_error <= stop_tolerance:
            limited_max_speed = min(limited_max_speed, final_min_speed)

        return max(-limited_max_speed, min(limited_max_speed, raw_angular_z))

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        return 2.0 * math.atan2(z, w)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _axis_heading(approach_axis: str, approach_sign: int) -> float:
        if approach_axis == "x":
            return 0.0 if approach_sign >= 0 else math.pi
        return math.pi * 0.5 if approach_sign >= 0 else -math.pi * 0.5


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapstoneTaskPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
