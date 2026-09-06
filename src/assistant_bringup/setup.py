from setuptools import setup

package_name = "assistant_bringup"
data_files = []

import os
from glob import glob

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (
        os.path.join("share", package_name, "launch"),
        glob("launch/*.launch.py"),
    ),
    (
        os.path.join("share", package_name, "config"),
        glob("config/*.yaml") + glob("config/*.json"),
    ),
]

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Multimodal Assistant Team",
    maintainer_email="team@example.com",
    description="System orchestration package: launch files and configuration.",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
