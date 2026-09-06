from setuptools import setup

package_name = "assistant_interaction"

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
    description="Human interaction package for voice, gaze and selection arbitration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "voice_node = assistant_interaction.voice_node:main",
            "gaze_node = assistant_interaction.gaze_node:main",
            "selection_manager_node = assistant_interaction.selection_manager_node:main",
        ],
    },
)
