from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'assistant_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='awtistic',
    maintainer_email='awtistic@todo.todo',
    description='URDF and RViz description for the 4-DOF robotic assistant arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
