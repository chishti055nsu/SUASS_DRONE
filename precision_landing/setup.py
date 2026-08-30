import os
from glob import glob
from setuptools import setup, find_packages

package_name = "precision_landing"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IUB Drone Team",
    maintainer_email="drone@iub.edu.bd",
    description="Deterministic ArUco / AprilTag Precision Landing & Payload Drop Package for SUAS Drone",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "precision_node = precision_landing.precision_node:main",
        ],
    },
)
