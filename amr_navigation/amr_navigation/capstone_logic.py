import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class RobotSpec:
    name: str
    prim_path: str
    pose: Pose3D


@dataclass(frozen=True)
class PalletSpec:
    name: str
    prim_path: str
    pose: Pose3D


@dataclass(frozen=True)
class PreDockProcessResult:
    blocker_name: str
    target_name: str
    approach_axis: str
    approach_sign: int
    pre_dock_pose: Pose3D
    pre_dock_yaw: float


PRIM_STRUCTURE_FILE = "/home/nsl/Capston_workspace/occupancy_map/maps/Map01_prim_structure.txt"
ROBOT_STRUCTURE_FILE = "/home/nsl/Capston_workspace/occupancy_map/maps/Map01_robot_structure.txt"

ROBOT_PRIM_PATHS = (
    "/iw_hub_ROS",
    "/iw_hub_ROS_01",
)

NAV2_NAMESPACE_BY_ROBOT = {
    "iw_hub_ROS": "iw_hub_ros",
    "iw_hub_ROS_01": "iw_hub_ros_01",
}


def parse_position(raw_value: str) -> Pose3D:
    values = raw_value.strip().removeprefix("(").removesuffix(")")
    x_str, y_str, z_str = [item.strip() for item in values.split(",")]
    return Pose3D(float(x_str), float(y_str), float(z_str))


def load_top_level_pallets(structure_file: str) -> Dict[str, PalletSpec]:
    pallets: Dict[str, PalletSpec] = {}
    path = Path(structure_file)
    if not path.exists():
        return pallets

    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        prim_path, _, position_text = line.partition("| position=")
        prim_path = prim_path.strip()
        pallet_name = Path(prim_path).name
        if not pallet_name.startswith("PalletBin_"):
            continue
        if not position_text:
            continue
        pallets[pallet_name] = PalletSpec(
            name=pallet_name,
            prim_path=prim_path,
            pose=parse_position(position_text.strip()),
        )
    return pallets


def load_robot_specs(structure_file: str) -> Dict[str, RobotSpec]:
    robots: Dict[str, RobotSpec] = {}
    path = Path(structure_file)
    if not path.exists():
        return robots

    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        prim_path, _, position_text = line.partition("| position=")
        prim_path = prim_path.strip()
        robot_name = Path(prim_path).name
        if not position_text:
            continue
        robots[robot_name] = RobotSpec(
            name=robot_name,
            prim_path=prim_path,
            pose=parse_position(position_text.strip()),
        )
    return robots


def load_named_prim_pose(structure_file: str, prim_path_query: str) -> Optional[Pose3D]:
    path = Path(structure_file)
    if not path.exists():
        return None

    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        prim_path, _, position_text = line.partition("| position=")
        prim_path = prim_path.strip()
        if prim_path != prim_path_query:
            continue
        if not position_text:
            return None
        return parse_position(position_text.strip())

    return None


def blocker_name_from_target(target_name: str) -> Optional[str]:
    if not target_name.endswith("_02"):
        return None
    return target_name[:-2] + "01"


def planar_distance(pose_a: Pose3D, pose_b: Pose3D) -> float:
    return math.hypot(pose_a.x - pose_b.x, pose_a.y - pose_b.y)


def select_closest_robot(
    robots: Dict[str, RobotSpec], target_pose: Pose3D
) -> Tuple[RobotSpec, float]:
    ranked = [
        (robot, planar_distance(robot.pose, target_pose))
        for robot in robots.values()
    ]
    ranked.sort(key=lambda item: item[1])
    return ranked[0]


def pre_dock_process_from_pair(
    blocker: PalletSpec,
    target: PalletSpec,
    pre_dock_distance: float,
    axis_tolerance: float,
) -> Optional[PreDockProcessResult]:
    same_y = abs(blocker.pose.y - target.pose.y) <= axis_tolerance
    same_x = abs(blocker.pose.x - target.pose.x) <= axis_tolerance

    if same_y and not same_x:
        if blocker.pose.x < target.pose.x:
            pre_dock_pose = Pose3D(
                blocker.pose.x - pre_dock_distance,
                blocker.pose.y,
                blocker.pose.z,
            )
            approach_sign = -1
        else:
            pre_dock_pose = Pose3D(
                blocker.pose.x + pre_dock_distance,
                blocker.pose.y,
                blocker.pose.z,
            )
            approach_sign = 1
        pre_dock_yaw = math.atan2(
            blocker.pose.y - pre_dock_pose.y,
            blocker.pose.x - pre_dock_pose.x,
        )
        return PreDockProcessResult(
            blocker_name=blocker.name,
            target_name=target.name,
            approach_axis="x",
            approach_sign=approach_sign,
            pre_dock_pose=pre_dock_pose,
            pre_dock_yaw=pre_dock_yaw,
        )

    if same_x and not same_y:
        if blocker.pose.y < target.pose.y:
            pre_dock_pose = Pose3D(
                blocker.pose.x,
                blocker.pose.y - pre_dock_distance,
                blocker.pose.z,
            )
            approach_sign = -1
        else:
            pre_dock_pose = Pose3D(
                blocker.pose.x,
                blocker.pose.y + pre_dock_distance,
                blocker.pose.z,
            )
            approach_sign = 1
        pre_dock_yaw = math.atan2(
            blocker.pose.y - pre_dock_pose.y,
            blocker.pose.x - pre_dock_pose.x,
        )
        return PreDockProcessResult(
            blocker_name=blocker.name,
            target_name=target.name,
            approach_axis="y",
            approach_sign=approach_sign,
            pre_dock_pose=pre_dock_pose,
            pre_dock_yaw=pre_dock_yaw,
        )

    return None
