from glob import glob
from setuptools import find_packages, setup

package_name = 'hydrone_confined'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivo Arpino',
    maintainer_email='ivoarpino2@gmail.com',
    description='Confined-space navigation with a 360 deg lidar (ROS-free core + thin nodes)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beam_localizer_node = hydrone_confined.nodes.beam_localizer_node:main',
            'pose_mux_node = hydrone_confined.nodes.pose_mux_node:main',
            'kinematic_sim_node = hydrone_confined.nodes.kinematic_sim_node:main',
            'viz_udp_node = hydrone_confined.nodes.viz_udp_node:main',
        ],
    },
)
