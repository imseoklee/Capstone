from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="amr_navigation",
                executable="initial_pose_publisher",
                name="initial_pose_publisher",
                output="screen",
                parameters=[
                    os.path.join(
                        "/home/nsl/Capston_workspace",
                        "amr_navigation",
                        "config",
                        "pallet_rules.yaml",
                    )
                ],
            ),
            Node(
                package="amr_navigation",
                executable="capstone_task_planner",
                name="capstone_task_planner",
                output="screen",
                parameters=[
                    os.path.join(
                        "/home/nsl/Capston_workspace",
                        "amr_navigation",
                        "config",
                        "pallet_rules.yaml",
                    )
                ],
            )
        ]
    )
