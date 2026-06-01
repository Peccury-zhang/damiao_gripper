from glob import glob
import os

from setuptools import find_packages, setup

package_name = "gripper_dm"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "gripper_dm": ["*.so"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="yclab",
    maintainer_email="yclab@example.com",
    description="ROS 2 gripper controller for DM-J4310-2EC via CANFD analyzer",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gripper_node = gripper_dm.gripper_node:main",
        ],
    },
)
