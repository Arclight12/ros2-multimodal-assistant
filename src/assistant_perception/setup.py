from setuptools import setup

package_name = "assistant_perception"

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
    description="Perception package for workspace understanding and object map generation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "workspace_mapper_node = assistant_perception.workspace_mapper_node:main",
        ],
    },
)
