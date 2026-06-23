from setuptools import find_packages
from setuptools import setup

setup(
    name='ardupilot_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('ardupilot_msgs', 'ardupilot_msgs.*')),
)
