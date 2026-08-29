from setuptools import setup, find_packages
import os
from glob import glob

package_name = "drone_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IUB Drone Team",
    maintainer_email="iub_drone@example.com",
    description="YOLO + Gemma drone vision ROS2 package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_node = drone_vision.vision_node:main",
        ],
    },
)
