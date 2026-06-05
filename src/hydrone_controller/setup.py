from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hydrone_controller'

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
    description='High-level drone controller (MAVROS bridge)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = hydrone_controller.controller_node:main',
        ],
    },
)
