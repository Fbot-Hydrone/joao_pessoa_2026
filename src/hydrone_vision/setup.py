from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hydrone_vision'

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
    description='Vision node: base detection, gesture recognition, QR reading',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = hydrone_vision.vision_node:main',
            'pad_detector_node = hydrone_vision.pad_detector_node:main',
        ],
    },
)
