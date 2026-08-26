from setuptools import setup, find_packages

package_name = 'hydrone_map'

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
    description='What the drone remembers about the world: the pad map and the accumulated point-cloud map',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pad_map_node = hydrone_map.pad_map_node:main',
            'feature_map_node = hydrone_map.feature_map_node:main',
        ],
    },
)
