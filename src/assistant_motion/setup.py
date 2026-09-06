from setuptools import setup

package_name = "assistant_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Multimodal Assistant Team",
    maintainer_email="team@example.com",
    description="Motion execution package for MoveIt integration and Arduino control.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "motion_planner_node = assistant_motion.motion_planner_node:main",
            "arm_controller_node = assistant_motion.arm_controller_node:main",
        ],
    },
)
