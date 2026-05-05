import importlib.util
import sys

import omni
from pxr import Usd, UsdGeom

CAPSTONE_LOGIC_PATH = (
    "/home/nsl/Capston_workspace/amr_navigation/amr_navigation/capstone_logic.py"
)

spec = importlib.util.spec_from_file_location("capstone_logic", CAPSTONE_LOGIC_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"Unable to load capstone logic from {CAPSTONE_LOGIC_PATH}")

capstone_logic = importlib.util.module_from_spec(spec)
sys.modules["capstone_logic"] = capstone_logic
spec.loader.exec_module(capstone_logic)

from capstone_logic import (  # type: ignore  # noqa: E402
    PRIM_STRUCTURE_FILE,
    ROBOT_PRIM_PATHS,
    ROBOT_STRUCTURE_FILE,
    Pose3D,
    PalletSpec,
    RobotSpec,
    blocker_name_from_target,
    select_closest_robot,
)


TARGET_PALLET = "PalletBin_A_02"
TOP_LEVEL_PREFIX = "/Root/"
TOP_LEVEL_DEPTH = 2


def load_stage_top_level_pallets(stage: Usd.Stage) -> dict[str, PalletSpec]:
    pallets: dict[str, PalletSpec] = {}
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if "PalletBin" not in path:
            continue
        if not path.startswith(TOP_LEVEL_PREFIX):
            continue
        if path.count("/") != TOP_LEVEL_DEPTH:
            continue

        transform = cache.GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        name = prim.GetName()
        pallets[name] = PalletSpec(
            name=name,
            prim_path=path,
            pose=Pose3D(
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            ),
        )
    return pallets


def load_stage_robot_specs(stage: Usd.Stage) -> dict[str, RobotSpec]:
    robots: dict[str, RobotSpec] = {}
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim_path in ROBOT_PRIM_PATHS:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            print(f"[ROBOT] Missing prim: {prim_path}")
            continue
        transform = cache.GetLocalToWorldTransform(prim)
        translation = transform.ExtractTranslation()
        name = prim.GetName()
        robots[name] = RobotSpec(
            name=name,
            prim_path=prim_path,
            pose=Pose3D(
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            ),
        )
        print(
            "[ROBOT] %s position=(%.6f, %.6f, %.6f)"
            % (
                prim_path,
                robots[name].pose.x,
                robots[name].pose.y,
                robots[name].pose.z,
            )
        )
    return robots


def save_top_level_pallets(pallets: dict[str, PalletSpec], output_path: str) -> None:
    lines = []
    for pallet in sorted(pallets.values(), key=lambda item: item.name):
        lines.append(
            "%s | position=(%.6f, %.6f, %.6f)"
            % (
                pallet.prim_path,
                pallet.pose.x,
                pallet.pose.y,
                pallet.pose.z,
            )
        )
    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write("\n".join(lines) + ("\n" if lines else ""))
    print(f"[SAVE] Wrote top-level pallet structure to {output_path}")


def save_robot_specs(robots: dict[str, RobotSpec], output_path: str) -> None:
    lines = []
    for robot in sorted(robots.values(), key=lambda item: item.name):
        lines.append(
            "%s | position=(%.6f, %.6f, %.6f)"
            % (
                robot.prim_path,
                robot.pose.x,
                robot.pose.y,
                robot.pose.z,
            )
        )
    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write("\n".join(lines) + ("\n" if lines else ""))
    print(f"[SAVE] Wrote robot structure to {output_path}")


def plan_target_blocker_assignment(
    pallets: dict[str, PalletSpec], robots: dict[str, RobotSpec], target_pallet: str
) -> None:
    target = pallets.get(target_pallet)
    if target is None:
        print(f"[PLAN] Target pallet {target_pallet} not found")
        return

    blocker_name = blocker_name_from_target(target_pallet)
    if blocker_name is None:
        print(f"[PLAN] Target pallet {target_pallet} does not follow the *_02 rule")
        return

    blocker = pallets.get(blocker_name)
    if blocker is None:
        print(f"[PLAN] Blocker pallet {blocker_name} not found")
        return

    if not robots:
        print("[PLAN] No AMR prims were found in the stage")
        return

    assigned_robot, distance = select_closest_robot(robots, blocker.pose)
    print(f"[PLAN] target={target.name} target_prim={target.prim_path}")
    print(f"[PLAN] blocker={blocker.name} blocker_prim={blocker.prim_path}")
    print(
        "[PLAN] blocker_position=(%.3f, %.3f, %.3f)"
        % (blocker.pose.x, blocker.pose.y, blocker.pose.z)
    )
    print(
        "[PLAN] assigned_robot=%s distance=%.3f"
        % (assigned_robot.prim_path, distance)
    )


def main() -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No stage is currently open in Isaac Sim.")

    pallets = load_stage_top_level_pallets(stage)
    robots = load_stage_robot_specs(stage)
    save_top_level_pallets(pallets, PRIM_STRUCTURE_FILE)
    save_robot_specs(robots, ROBOT_STRUCTURE_FILE)
    plan_target_blocker_assignment(pallets, robots, TARGET_PALLET)
    print("[DONE] Capstone main completed.")


main()
