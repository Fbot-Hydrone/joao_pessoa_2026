from setuptools import setup, find_packages

package_name = 'hydrone_localization'

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
    description='Where the drone thinks it is: visual odometry, the map->odom edge, and the pose fed to ArduPilot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'visual_odometry_node = hydrone_localization.visual_odometry_node:main',
            'map_odom_node = hydrone_localization.map_odom_node:main',
            'vision_odom_bridge = hydrone_localization.vision_odom_bridge:main',
            'landmark_correction_node = hydrone_localization.landmark_correction_node:main',
        ],
    },
)
