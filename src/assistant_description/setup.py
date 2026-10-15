import os
from glob import glob

from setuptools import find_packages, setup

package_name = "assistant_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Multimodal Assistant Team",
    maintainer_email="team@example.com",
    description="URDF and RViz description for the 4-DOF robotic assistant arm",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
