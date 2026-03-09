from setuptools import find_packages, setup
import os
from glob import glob

package_name = "warehouse_robot_navigation_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),          # ← fixes marker warning
        ("share/" + package_name, ["package.xml"]), # ← fixes package.xml warning
    ],
    entry_points={
        "console_scripts": [
            "warehouse_mission_executor = warehouse_robot_navigation_py.warehouse_mission_executor:main",
        ],
    },
)