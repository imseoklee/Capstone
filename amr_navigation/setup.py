from glob import glob
import os

from setuptools import setup


package_name = "amr_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nsl",
    maintainer_email="nsl@todo.todo",
    description="ROS 2 Python package for capstone AMR navigation and pallet task sequencing.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "initial_pose_publisher = amr_navigation.initial_pose_publisher:main",
            "capstone_task_planner = amr_navigation.capstone_task_planner:main",
        ],
    },
)
