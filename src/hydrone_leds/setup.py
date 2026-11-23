from setuptools import setup, find_packages
from glob import glob

package_name = 'hydrone_leds'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hydrone Team',
    maintainer_email='team@hydrone.com',
    description='On/off RGB LED strip on three GPIO-driven MOSFETs (camera lighting)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leds = hydrone_leds.cli:main',
            'led_node = hydrone_leds.led_node:main',
        ],
    },
)
