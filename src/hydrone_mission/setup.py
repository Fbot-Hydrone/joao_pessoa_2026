from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hydrone_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hydrone Team',
    maintainer_email='team@hydrone.com',
    description='Top-level mission state machine for all 4 competition phases',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_node = hydrone_mission.mission_node:main',
            'pad_mission_node = hydrone_mission.pad_mission_node:main',
        ],
    },
)
