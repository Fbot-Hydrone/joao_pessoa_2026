from glob import glob
from setuptools import setup

package_name = 'hydrone_lio'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivo Arpino',
    maintainer_email='ivoarpino2@gmail.com',
    description='Phase 4 LIO glue nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lio_odom_adapter = hydrone_lio.lio_odom_adapter:main',
            'motion_prior_node = hydrone_lio.motion_prior_node:main',
            'lio_map_node = hydrone_lio.lio_map_node:main',
        ],
    },
)
