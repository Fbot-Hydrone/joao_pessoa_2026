from setuptools import setup
import os
from glob import glob

package_name = 'hydrone_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'params'),
            glob('config/params/*.parm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hydrone Team',
    maintainer_email='team@hydrone.com',
    description='Launch files and configuration for the Hydrone competition stack',
    license='MIT',
    entry_points={'console_scripts': [
        'zed_mimic_node = hydrone_bringup.zed_mimic_node:main',
        'vision_odom_bridge = hydrone_bringup.vision_odom_bridge:main',
        'visual_odometry_node = hydrone_bringup.visual_odometry_node:main',
    ]},
)
