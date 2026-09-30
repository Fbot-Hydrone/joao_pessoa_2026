from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hydrone_nav'

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
    description='Navigation: route planning, precision landing, maze traversal',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nav_node = hydrone_nav.nav_node:main',
            'pad_map_node = hydrone_nav.pad_map_node:main',
            'feature_map_node = hydrone_nav.feature_map_node:main',
        ],
    },
)
