from setuptools import find_packages
from setuptools import setup

setup(
    name='ardupilot_sitl',
    version='0.0.0',
    packages=find_packages(
        include=('ardupilot_sitl', 'ardupilot_sitl.*')),
)
